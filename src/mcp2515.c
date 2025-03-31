#include "mcp2515_circular_buffer.h"
#include "../common/mcp2515_ioctl.h"
#include "mcp2515.h"

#include <linux/module.h>		// required for module initialization and cleanup macros
#include <linux/init.h>			// required for module initialization and exit macros
#include <linux/kernel.h>		// required for kernel functions like pr_info
#include <linux/of.h>			// required for device tree interaction
#include <linux/of_device.h>	// requiered for device and driver matching via the device tree 
#include <linux/delay.h>
#include <linux/spinlock.h>     // required for spinlock 
#include <linux/slab.h>         // required for memory allocation and management
#include <linux/of_irq.h>       // requred to parse the interrupt from device tree
#include <linux/interrupt.h>    // interrupt for interrupt handling 
#include <linux/wait.h>         // requered for wait queue definitions and functions
#include <linux/sched.h>        // required for scheduling functions and task structure
#include <linux/workqueue.h>    // required for work queue definitions and functions
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/minmax.h>
#include <linux/fs.h>           // Provides file system operations, including character device registeration functions 
#include <linux/cdev.h>         // Required for managing character devices 
#include <linux/uaccess.h>      // Provides user space memory access functions
#include <linux/kdev_t.h>       // Requred for device identification macros
#include <linux/device.h>       // Provides definitions for class related functions 
#include <linux/atomic.h>       // Provides atomic types and operations 
#include <linux/poll.h>
#include <linux/spi/spi.h>


#define MCP2515_DEVICE_NAME     "mcp2515"           // device name 
#define MCP2515_CLASS           "mcp2515_class"     // device class name         

// configuration parameters of MCP2515 hardware 
struct mcp2515_platform_data {
    // general parameters:
    u32 osc_freq;   // Oscillator frequency in Hz
    u32 ost;        // Oscillator Start-up Timer in clock cycles 
    // interupt parameters:
    int irq;
    // CAN-Bus parameters:
    u32 baudrate;   // CAN bus baud rate in bps (back-calculated value)
    u8 brp;         // Baudrate prescaler of main oscillator clock 
    u8 tsync;       // Synchronization segment time in TQ
    u8 tprop_seg;   // Propagation segment time in TQ
    u8 ps1;         // Phase segment 1 time in TQ
    u8 ps2;         // Phase segment 2 time in TQ
    u8 sjw;         // Synchronisation jump width in TQ
    u8 btlmode;     // PS2 Bit Time Length bit
    u8 sam;         // Sample Point Configuration bit
};

struct mcp2515_work_data {
    struct delayed_work work;   // Delayed work structure for deferred execution
    unsigned long delay_ms;     // Delay in milliseconds for rescheduling work
    unsigned long timeout_ms;   // Timeout in milliseconds for delayed work
};

enum mcp2515_condition {
    MCP2515_COND_IDLE = 0,
    MCP2515_COND_SUCCESS,
    MCP2515_COND_FAILURE
};

// driver specific data 
struct mcp2515_priv {
    struct mcp2515_platform_data *pdata;            // Pointer to the platform-specific data
    struct spi_device *spi;                         // Pointer to the SPI device structure
    struct mutex spi_lock;                          // Mutex pt protect the spi from race conditions 

    spinlock_t lock;                                // Spinlock for protecting shared resources (need ?)
    wait_queue_head_t opmode_wait_queue;            // Wait queue for synchronisation with condition changes
    enum mcp2515_condition opmode_condition;        // Condition variable indicating the curent state     
    struct workqueue_struct *opmode_work_queue;     // Workqueue for scheduling work items 
    struct mcp2515_work_data opmode_work_data;      // Structure for delayed work and timing parameters
    u8 des_opmode;                                  // Desired operation mode for MCP2515
    int opmode_err;                                 // Error code indicating the result of the last operation

    struct circular_buffer rx_cbuf;                 // Circular buffer for recieving messages
    struct circular_buffer tx_cbuf;                 // Circular buffer for transmiting messages 

    wait_queue_head_t rx_wait_queue;                // Wait queue to synchrnize data reception
    wait_queue_head_t tx_wait_queue;                // Wait queue to synchronize data transmission 
    struct workqueue_struct *tx_work_queue;         // Work queue to trnasmit CAN messages from circular buffer tom MCP2515 TX2 buffer
    struct work_struct tx_work;                     // Work to transmit CaN message from circular buffer to MCP2515 TX2 buffer           
    atomic_t tx_buf_ready;                          // Indicates if the MCP2515 TX2 Buffer is ready for new data 

    struct cdev mcp2515_cdev;
    struct class *mcp2515_class;
    dev_t dev_num;
};

// entry definition for interrupt lookup table 
typedef int (*mcp2515_irq_handler_t)(struct mcp2515_priv *);   // define function pointer for interrupt handlers
struct mcp2515_irq_entry {
    u8 flag;
    mcp2515_irq_handler_t handler;
};

/* forward declarations: */
static int mcp2515_write_reg(struct spi_device *spi, u8 reg, u8 value);
static int mcp2515_read_reg(struct spi_device *spi, u8 reg, u8 *value);
static int mcp2515_write_bit(struct spi_device *spi, u8 reg, u8 mask, u8 value);

static int mcp2515_set_opmode(struct spi_device *spi, u8 mode);
static int mcp2515_get_opmode(struct spi_device *spi, u8 *mode);
static int mcp2515_reset_hw(struct spi_device *spi);
static int mcp2515_config_hw(struct spi_device *spi);
static int mcp2515_start_dwork(struct mcp2515_priv *priv, work_func_t work_func);
static void mcp2515_check_opmode_work(struct work_struct *work);

static int mcp2515_set_tx2_id(struct spi_device *spi, u16 id);                                  // set the standard indentifier for the transmit buffer 2
static int mcp2515_set_tx2_data(struct spi_device *spi, const u8 *data, u8 len);                // set the message data for the transmit buffer 2
static int mcp2515_write_can_frame(struct spi_device *spi, u16 id, const u8 *data, u8 len);     // transmit the CAN messsage over the transmit buffer 2
static void mcp2515_write_can_frame_work(struct work_struct *work);                             // work function tor transmit CAN Messages

static int mcp2515_get_rx1_id(struct spi_device *spi, u16 *id);                                 // get standard identifier from the receive buffer 1
static int mcp2515_get_rx1_data(struct spi_device *spi, u8 *data, u8 *len);                     // get the message data and lenght from the receive buffer 1
static int mcp2515_read_can_frame(struct spi_device *spi, u16 *id, u8 *data, u8 *dlc);          // receive the CAN message over the receive buffer 1

// interrupt handling routines 
static irqreturn_t mcp2515_irq(int irq, void *dev_id);          // irq top half
static irqreturn_t mcp2515_irq_handler(int irq, void *dev_id);  // irq bottom half 
static int mcp2515_handle_merrf(struct mcp2515_priv *priv);
static int mcp2515_handle_wakif(struct mcp2515_priv *priv);
static int mcp2515_handle_errif(struct mcp2515_priv *priv);
static int mcp2515_handle_tx2if(struct mcp2515_priv *priv);
static int mcp2515_handle_tx1if(struct mcp2515_priv *priv);
static int mcp2515_handle_tx0if(struct mcp2515_priv *priv);
static int mcp2515_handle_rx1if(struct mcp2515_priv *priv);
static int mcp2515_handle_rx0if(struct mcp2515_priv *priv);

// character device file operations 
static int mcp2515_open(struct inode *inode, struct file *file);
static int mcp2515_release(struct inode *inode, struct file *file);
static unsigned int mcp2515_poll(struct file *file, poll_table *wait); 
static ssize_t mcp2515_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t mcp2515_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos);
static long mcp2515_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int mcp2515_set_rxbm(struct spi_device *spi, struct rx_mask mask);
static int mcp2515_set_rxbf(struct spi_device *spi, struct rx_filter filter);
static int mcp2515_get_rxbf(struct spi_device *spi, struct rx_filter *filter);

// device provisioning 
static int mcp2515_spi_probe(struct spi_device *spi);
static void mcp2515_spi_remove(struct spi_device *spi);

// device file operation exposed to the userspace
static struct file_operations mcp2515_fops = {
    .owner = THIS_MODULE,
    .open = mcp2515_open,
    .release = mcp2515_release,
    .poll = mcp2515_poll,
    .read = mcp2515_read,
    .write = mcp2515_write,
    .unlocked_ioctl = mcp2515_ioctl,
} ;

// interrupt handler lookup table
static struct mcp2515_irq_entry irq_table[] = {
    { MCP2515_MERRF, mcp2515_handle_merrf },
    { MCP2515_WAKIF, mcp2515_handle_wakif },
    { MCP2515_ERRIF, mcp2515_handle_errif },
    { MCP2515_TX2IF, mcp2515_handle_tx2if },
    { MCP2515_TX1IF, mcp2515_handle_tx1if },
    { MCP2515_TX0IF, mcp2515_handle_tx0if },
    { MCP2515_RX1IF, mcp2515_handle_rx1if },
    { MCP2515_RX0IF, mcp2515_handle_rx0if },
};

static int mcp2515_write_reg(struct spi_device *spi, u8 reg, u8 value) 
{
    u8 tx_buf[3];
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = 3,
    };
    struct spi_message m;

    tx_buf[0] = MCP2515_WRITE;
    tx_buf[1] = reg;
    tx_buf[2] = value;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return spi_sync(spi, &m);
}

static int mcp2515_read_reg(struct spi_device *spi, u8 reg, u8 *value)
{
	int ret = 0;
    u8 tx_buf[2];
    u8 rx_buf[1];

    struct spi_transfer t[] = {
        {
            .tx_buf = tx_buf,
            .len = 2,
        },
        {
            .rx_buf = rx_buf,
            .len = 1,
        },
    };
    struct spi_message m;
	
    tx_buf[0] = MCP2515_READ;
    tx_buf[1] = reg;

    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);
    ret = spi_sync(spi, &m);
    if (ret == 0)
        *value = rx_buf[0];

    return ret;
}

static int mcp2515_write_bit(struct spi_device *spi, u8 reg, u8 mask, u8 value)
{
    u8 tx_buf[4];
    struct spi_transfer t = {
        .tx_buf = tx_buf,
        .len = 4,
    };
    struct spi_message m;

    tx_buf[0] = MCP2515_BIT_MODIFY;
    tx_buf[1] = reg;
    tx_buf[2] = mask;
    tx_buf[3] = value;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    return spi_sync(spi, &m);
}

static int mcp2515_set_opmode(struct spi_device *spi, u8 mode)
{
    int ret = 0;
    u8 canctl = 0x00;           // CAN-Control register value 
    struct mcp2515_priv *priv = spi_get_drvdata(spi);
    priv->des_opmode = (mode << MCP2515_OPMOD_O) & MCP2515_OPMOD_M;    // desired operation mode

    ret = mcp2515_read_reg(spi, MCP2515_CANCTRL, &canctl);
    if (ret) {
        dev_err(&spi->dev, "Failed to read CAN-Control register\n");
        return ret;
    }

    // clear REQOP bits and set the desired operation mode
    canctl = ((canctl & ~MCP2515_OPMOD_M) | priv->des_opmode);

    ret = mcp2515_write_reg(spi, MCP2515_CANCTRL, canctl);
    if (ret) {
        dev_err(&spi->dev, "Failed to write to operation mode register\n");
        return ret;
    }

    return mcp2515_start_dwork(priv, mcp2515_check_opmode_work); 
}

static int mcp2515_get_opmode(struct spi_device *spi, u8 *mode)
{
    int ret = 0;
    u8 canstat = 0x0;

    // read CANSTAT register
    ret = mcp2515_read_reg(spi, MCP2515_CANSTAT, &canstat);
    if (ret) {
        dev_err(&spi->dev, "Failed to read CANSTAT register\n");
        return -EFAULT; 
    }

    // retrive operation mode from the status register 
    *mode = ((canstat & MCP2515_OPMOD_M) >> MCP2515_OPMOD_O); 

    return 0;
}

static int mcp2515_reset_hw(struct spi_device *spi) 
{
    int ret = 0;
    struct mcp2515_priv *priv = spi_get_drvdata(spi);
    u8 mode = MCP2515_CONFIG_MODE << MCP2515_OPMOD_O;
    u8 reset_cmd = MCP2515_RESET;  // set the desired command
    
    // create spi message 
    struct spi_transfer transfer = {
        .tx_buf = &reset_cmd,
        .len = 1,
    };
    struct spi_message message;

    spi_message_init(&message);
    spi_message_add_tail(&transfer, &message);

    ret = spi_sync(spi, &message);
    if (ret) {
        dev_err(&spi->dev, "Failed to write reset command\n");
        return ret;
    }

    priv->des_opmode = mode;    // after reset the hardware should be in the configuration mode 
    return mcp2515_start_dwork(priv, mcp2515_check_opmode_work);    // wait for the set delay time and check the operation mode
}

static int mcp2515_config_hw(struct spi_device *spi)
{
    int ret = 0;     // return value on success	 
    u8 cnf1 =  0;    // baudrate register 1 value
    u8 cnf2 = 0;     // baudrate register 2 value
    u8 cnf3 = 0;     // baudrate register 3 value
    u8 caninte = 0;  // interrupt register value
    u8 rxb0ctrl = 0; // receive buffer 0 control register value   
    u8 rxb1ctrl = 0; // receive buffer 1 control register value

    struct rx_mask mask;
    struct rx_filter filter;
    struct mcp2515_platform_data *pdata = NULL;
    struct mcp2515_priv *priv = spi_get_drvdata(spi);
    if(!priv) {
        dev_err(&spi->dev, "Failed to get privat data\n");
        return -EINVAL;
    }

    pdata = priv->pdata;
    if(!pdata) {
        dev_err(&spi->dev, "Error in mcp2515_config_hw(): pdata pointer is null\n");
        return -EINVAL;
    }

    // set bittiming, sample point and baudrate via (CNF1, CNF2, CNF3) registers
    cnf1 |= ((((pdata->sjw - 1) << MCP2515_SJW_O) & MCP2515_SJW_M) | ((pdata->brp - 1) & MCP2515_BRP_M));
    cnf2 |= (pdata->btlmode << MCP2515_BTLMODE_O) | (pdata->sam << MCP2515_SAM_O) | ((pdata->ps1 -1 ) << MCP2515_PHSEG1_O) | ((pdata->tprop_seg - 1) << MCP2515_PRSEG_O);
    cnf3 |= (pdata->ps2 - 1);

    ret = mcp2515_write_reg(spi, MCP2515_CNF1, cnf1);
    ret |= mcp2515_write_reg(spi, MCP2515_CNF2, cnf2);
    ret |= mcp2515_write_reg(spi, MCP2515_CNF3, cnf3);
    if (ret) {
        dev_err(&spi->dev, "Failed to set baud rate registers\n");
        return ret;
    }
    
    // enable interrupts 
    caninte = 0xFF; // enable all interrupts 
    ret = mcp2515_write_reg(spi, MCP2515_CANINTE, caninte);
    if (ret) {
        dev_err(&spi->dev, "Failed to set interrupt enable register\n");
        return ret;
    }

    // configure receive RxB0: no rollover, ingnore all messages by seting filter and mask to 0x7FF
    rxb0ctrl |= (MCP2515_RXM_RCV_VALID_MSG << MCP2515_RXB0_RXM_O); // set the operation mode to receives all valid messages 
    ret |= mcp2515_write_reg(spi, MCP2515_RXB0CTRL, rxb0ctrl);
    // set mask for RxB0: match all bits to filter CAN ID
    mask.number = 0;
    mask.value = 0x7FF;  // 0111 1111 1111 matching mask
    ret |= mcp2515_set_rxbm(spi, mask);
    // set rxb0 filter 0 to: 0x7FF
    filter.value =  0x7FF;  // 1111 1111 filter vlaue 
    filter.number = 0;
    ret |= mcp2515_set_rxbf(spi, filter);
    // set rxb0 filter 1 to: 0x7FF
    filter.number = 1;
    ret |= mcp2515_set_rxbf(spi, filter);
    if (ret) {
        dev_err(&spi->dev, "Failed to configure receive buffer 0\n");
        return ret;
    }

    // configure RxB1: receives all valid standard messages in range [0x123:0x125]
    rxb1ctrl |= (MCP2515_RXM_RCV_VALID_MSG << MCP2515_RXB1_RXM_O); // set the operation mode to: receives all valid messages
    ret |= mcp2515_write_reg(spi, MCP2515_RXB1CTRL, rxb1ctrl);
    // set mask for RxB1: match all bits to filter CAN ID
    mask.number = 1;
    mask.value =  0x7FF;
    ret |= mcp2515_set_rxbm(spi, mask);
    // set rxb1 filter 2 to 5 to the value: 0x00
    filter.value = 0x00;
    for (filter.number = 2; filter.number <=5; filter.number++) {
        ret |= mcp2515_set_rxbf(spi, filter);
    }
    if (ret) {
        dev_err(&spi->dev, "Failed to configure receive buffer 1\n");
        return ret;
    }

    return ret;
}

static int mcp2515_start_dwork(struct mcp2515_priv *priv, work_func_t work_func)
{
    int ret = 0;                          
    priv->opmode_condition = MCP2515_COND_IDLE;  // init the condition variable

    // initialize the delayed work structure
    INIT_DELAYED_WORK(&priv->opmode_work_data.work, work_func);

    // queue the initial work with delay interval in milliseconds 
    queue_delayed_work(priv->opmode_work_queue, &priv->opmode_work_data.work, msecs_to_jiffies(priv->opmode_work_data.delay_ms));

    // wait for the condition with the timeout
    ret = wait_event_interruptible_timeout(priv->opmode_wait_queue, priv->opmode_condition != MCP2515_COND_IDLE, priv->opmode_work_data.timeout_ms);
    if (ret == 0) {
        dev_err(&priv->spi->dev, "Timeout occured, wait condition not met\n");
        cancel_delayed_work_sync(&priv->opmode_work_data.work);  // cancel pending work if timeout occurs
        return -EBUSY;
    } else if ( ret < 0) {
        dev_err(&priv->spi->dev, "Wait interrupted\n");
        cancel_delayed_work_sync(&priv->opmode_work_data.work);  // cancel pending work if interrupted
        return ret;
    }

    if (priv->opmode_condition != MCP2515_COND_SUCCESS) {
        return priv->opmode_err;   // failure 
    }

    return 0;   // success 
}

static void mcp2515_check_opmode_work(struct work_struct *work)
{   
    u8 canstat = 0; // CANSTAT register value
    unsigned long flags; // interrupt flags
    struct mcp2515_priv *priv = NULL;
    struct spi_device *spi = NULL;

    // obtain the work_data structure from the work_struct
    struct mcp2515_work_data *work_data = container_of(to_delayed_work(work), struct mcp2515_work_data, work);
    if (!work_data) {
        pr_err("Failed to get work_data structure\n");
        return;
    }

    // obtain the priv structure from the work_data structure
    priv = container_of(work_data, struct mcp2515_priv, opmode_work_data);
    if (!priv) {
        pr_err("Failed to get priv structure\n");
        return;
    }
    spi = priv->spi;

    // accuiere the spinlock to protect shared resources
    spin_lock_irqsave(&priv->lock, flags);

    // read CANSTAT register
    priv->opmode_err = mcp2515_read_reg(spi, MCP2515_CANSTAT, &canstat);
    if (priv->opmode_err) {
        dev_err(&spi->dev, "Failed to read CANSTAT register\n");
        priv->opmode_condition = MCP2515_COND_FAILURE;      // set the condition to indicate failure
        wake_up_interruptible(&priv->opmode_wait_queue);    // wake up waiting process 
        spin_unlock_irqrestore(&priv->lock, flags);         // release the spinlock and resotre the interrupt flags (need ?)
        return;
    } else if (((canstat & MCP2515_OPMOD_M)) == priv->des_opmode) {
        dev_info(&spi->dev, "Operation mode: %d set succesfully\n", (priv->des_opmode >> MCP2515_OPMOD_O));
        priv->opmode_condition = MCP2515_COND_SUCCESS;           // set the condition to indicate success
        wake_up_interruptible(&priv->opmode_wait_queue);    // wake up waiting process
        spin_unlock_irqrestore(&priv->lock, flags);         // release the lock and restore the interrupt flags
        return;
    }

    // schedule the next check after delay_ms interval 
    queue_delayed_work(priv->opmode_work_queue, &priv->opmode_work_data.work, msecs_to_jiffies(priv->opmode_work_data.delay_ms));

    // release the spinlock
    spin_unlock_irqrestore(&priv->lock, flags);
}

static int mcp2515_set_tx2_id(struct spi_device *spi, u16 id)
{
    int ret = 0;
    // standard Identifier consists of 11-bits
    u8 sidl, sidh;  // 3-lower and 8-higher bits 

    // Validate the 11-bit CAN ID
    if (id > 0x7FF) {
        dev_err(&spi->dev, "Invalid CAN SID: 0x%X. Must be 11 bits.\n", id);
        return -EINVAL;
    }
    
    // split the ID on 8-bit high and 3-bit low register
    sidl = (u8)(id << MCP2515_TXB_SIDL_O);  // save the lower 3-bits in the low register by appropriate shifting 
    sidh = (u8)(id >> 3);                   // save the upper 8-bits in the high register 
    
    ret = mcp2515_write_reg(spi, MCP2515_TXB2SIDL, sidl);
    ret |= mcp2515_write_reg(spi, MCP2515_TXB2SIDH, sidh);

    if (ret) {
        dev_err(&spi->dev, "Failed to set message SID: 0x%X for TXB2\n", id);
    }

    return ret;
}

static int mcp2515_set_tx2_data(struct spi_device *spi, const u8 *data, u8 len)
{
    int ret = 0;

    // limit the length to the maximum allowed value 
    u8 dlc = (len & MCP2515_TXB_DLC_M); 
    dlc = min(dlc, (u8)MCP2515_MAXDL);  // mask DLC to ensure proper format

    // set the data length code
    ret |= mcp2515_write_reg(spi, MCP2515_TXB2DLC, dlc);
    if (ret) {
        dev_err(&spi->dev, "Failed to set the data length code\n");
        return ret;
    }

    // write data bytes to the transmit buffer
    for (int i = 0; i < dlc; i++) {
        ret |= mcp2515_write_reg(spi, MCP2515_TXB2D0 + i, data[i]);
    } 

    if (ret) {
        dev_err(&spi->dev, "Failed to write data to TXB2\n");
    }

    return ret;
}

// transmit low priority message over the transmit buffer 2 
static int mcp2515_write_can_frame(struct spi_device *spi, u16 id, const u8 *data, u8 len)
{
    int ret = 0;
    u8 txb2ctrl = 0;

    txb2ctrl |= MCP2515_TXB_TXREQ;  // set transmit request
    txb2ctrl |= MCP2515_TXP_L_PRIO; // set message priority to low 
    
    ret = mcp2515_set_tx2_id(spi, id);
    ret |= mcp2515_set_tx2_data(spi, data, len);

    if (!ret) {
        // initiate message transmition
        ret |= mcp2515_write_reg(spi, MCP2515_TXB2CTRL, txb2ctrl);
    }

    return ret;
}

void mcp2515_write_can_frame_work(struct work_struct *work)
{
    struct mcp2515_priv *priv = container_of(work, struct mcp2515_priv, tx_work);
    struct spi_device *spi = priv->spi;
    struct can_frame_data frame;
    int ret = 0;

    while (!circular_buffer_is_empty(&priv->tx_cbuf)) {
        // wait for MCP2515 TX2 buffer to be ready for new data 
        ret = wait_event_interruptible_timeout(priv->tx_wait_queue, (atomic_read(&priv->tx_buf_ready) > 0), msecs_to_jiffies(100));
        if (ret == 0) {
            dev_err(&priv->spi->dev, "Timeout waiting MCP2515 TX2 buffer availability\n");
            return;
        } else if (ret < 0) {
            dev_err(&spi->dev, "Interrupted while waiting MCP2515 TX2 buffer availability\n");
            return;
        }

        // read the CAN message from the circular buffer 
        ret = circular_buffer_read(&priv->tx_cbuf, &frame);
        if (ret) {
            dev_err(&spi->dev, "Failed to read transmit circular buffer\n");
            return;
        }

        // transmit the CAN message
        mutex_lock(&priv->spi_lock);
        ret = mcp2515_write_can_frame(spi, frame.sid, frame.data, frame.dlc);
        mutex_unlock(&priv->spi_lock);
        if (ret) {
            dev_err(&spi->dev, "Failed to write CAN message to MCP2515\n");
            return;
        }

        // reset the TX2 buffer ready flag
        atomic_set(&priv->tx_buf_ready, 0);
    }
}

static int mcp2515_get_rx1_id(struct spi_device *spi, u16 *id)
{
    int ret = 0;  // return value 
    u8 sidh = 0;  // standard identifier high register value
    u8 sidl= 0;   // standard identifier low register value 
    *id = 0;       // init the output value for standard identifier      

    // read the 11-bit standard identifier 
    ret |= mcp2515_read_reg(spi, MCP2515_RXB1SIDH, &sidh); 
    ret |= mcp2515_read_reg(spi, MCP2515_RXB1SIDL, &sidl);
    if (ret) {
        dev_err(&spi->dev, "Failed to read RxB1 Standard Idetifier\n");
        return ret;
    }

    *id = ((u16)sidh << 3) | ((sidl & MCP2515_RXB_SID_M) >> MCP2515_RXB_SID_O); 
    return ret;
}

static int mcp2515_get_rx1_data(struct spi_device *spi, u8 *data, u8 *len)
{
    int ret = 0;

    ret |= mcp2515_read_reg(spi, MCP2515_RXB1DLC, len);  // read the data length in bytes 
    if (ret) {
        dev_err(&spi->dev, "Failed to read RxB1 data length\n");
        return ret;
    }

    for (int i = 0; i < *len; i++) {
        ret |= mcp2515_read_reg(spi, MCP2515_RXB1D0 + i, &data[i]);
    }

    if (ret) {
        dev_err(&spi->dev, "Failed to read RxB1 data\n");
        return ret;
    }

    return ret;
}

static int mcp2515_read_can_frame(struct spi_device *spi, u16 *id, u8 *data, u8 *dlc)
{
    int ret = 0;

    // read the standatrd idetifier, data and data length from receive buffer 1
    ret |= mcp2515_get_rx1_id(spi, id);
    ret |= mcp2515_get_rx1_data(spi, data, dlc);
    if (ret) {
        dev_err(&spi->dev, "Failed to read message from RxB1\n");
        return ret;
    }

    return ret;
}

static irqreturn_t mcp2515_irq(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD; // wake up the threaded IRQ handler  
}

static irqreturn_t mcp2515_irq_handler(int irq, void *dev_id)
{
    u8 ret = 0;
    struct mcp2515_priv *priv = dev_id;
    struct spi_device *spi = priv->spi;
    u8 canintf = 0;

    // read mcp2515 interrupt flag register 
    ret = mcp2515_read_reg(spi, MCP2515_CANINTF, &canintf);
    if (ret) {
        dev_err(&spi->dev, "Failed to read interrupt flag register\n");
        return ret;
    }

    // process the interrupts 
    for (int i = 0; i < ARRAY_SIZE(irq_table); i++) {
        if (canintf & irq_table[i].flag) {
            ret |= irq_table[i].handler(priv);
        }
    }

    if (ret) {
        dev_err(&spi->dev, "Failed to handle interrupts\n");
        return IRQ_NONE;
    }
    
    return IRQ_HANDLED;
}

static int mcp2515_handle_merrf(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "Message error interrupt occurred");
    ret = mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_MERRF, 0x0);  // clear the interrupt flag 
    return ret;
}

static int mcp2515_handle_wakif(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "Wake-up interrupt occurred");
    ret = mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_WAKIF, 0x0);  // clear the interrupt flag 
    return ret;
}

static int mcp2515_handle_errif(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "Error interrupt occurred");
    ret = mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_ERRIF, 0x0);  // clear the interrupt flag 
    return ret;
}

static int mcp2515_handle_tx2if(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "TX2 interrupt occurred");
    
    mutex_lock(&priv->spi_lock);
    ret |= mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_TX2IF, 0x0);  // clear the interrupt flag in the device 
    mutex_unlock(&priv->spi_lock);


    atomic_set(&priv->tx_buf_ready, 1);             // signal that the MCP2515 TX2 buffer is ready for new data 
    wake_up_interruptible(&priv->tx_wait_queue);    // wake up waiting functions 

    // schedule the transmit work queue
    queue_work(priv->tx_work_queue, &priv->tx_work);

    return ret;
}

static int mcp2515_handle_tx1if(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "TX1 interrupt occurred");
    ret = mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_TX1IF, 0x0);  // clear the interrupt flag in the device 
    return ret;
}

static int mcp2515_handle_tx0if(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "TX0 interrupt occurred");
    ret = mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_TX0IF, 0x0);  // clear the interrupt flag 
    return ret;
}

static int mcp2515_handle_rx1if(struct mcp2515_priv *priv)
{
    int ret = 0;
    struct spi_device *spi = priv->spi;
    struct can_frame_data frame;

    dev_info(&spi->dev, "RX1 interrupt occurred");

    // read the message (sid, dlc, data) from device buffer into privat buffer 
    mutex_lock(&priv->spi_lock);
    ret = mcp2515_read_can_frame(spi, &frame.sid, frame.data, &frame.dlc);
    mutex_unlock(&priv->spi_lock);
    if (ret) {
        dev_err(&spi->dev, "Failed to read CAN-Message\n");
        return -EFAULT;
    }

    ret = circular_buffer_write(&priv->rx_cbuf, &frame);
    if (ret) {
        // just signal ring buffer overflow (because it is valid condtion)
        dev_info(&spi->dev, "Receive buffer overflow: tail message was overriden\n");
    }
 
    mutex_lock(&priv->spi_lock);
    ret = mcp2515_write_bit(spi, MCP2515_CANINTF, MCP2515_RX1IF, 0x0);  // clear the interrupt flag
    mutex_unlock(&priv->spi_lock);
    if (ret) {
        dev_err(&spi->dev, "Failed to clear the RX1 interrupt flag\n");
        return -EFAULT;
    }

    wake_up_interruptible(&priv->rx_wait_queue);    // wake up all readers 

    return ret;
}

static int mcp2515_handle_rx0if(struct mcp2515_priv *priv)
{
    int ret = 0;
    dev_info(&priv->spi->dev, "RX0 interrupt occurred");
    ret = mcp2515_write_bit(priv->spi, MCP2515_CANINTF, MCP2515_RX0IF, 0x0);  // clear the interrupt flag 
    return ret;
}

static int mcp2515_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    struct mcp2515_priv *priv = NULL;
    struct spi_device *spi = NULL;
    unsigned int maj = imajor(inode);
    unsigned int min = iminor(inode);

    /* input parameter check */
    priv = container_of(inode->i_cdev, struct mcp2515_priv, mcp2515_cdev);
    if (!priv) {
        pr_err("Failed to retireve privat data from inode\n");
        return -EINVAL;
    }

    if (maj != MAJOR(priv->dev_num)  || min < 0) {
        pr_err("Device not found\n");
        return - ENODEV;
    }

    file->private_data = priv;   // bind privat data to the open file 
    spi = priv->spi;

    pr_info("Device Opened\n");
    return ret;
}

static int mcp2515_release(struct inode *inode, struct file *file)
{
    int ret = 0;
    struct mcp2515_priv *priv = NULL;

    // retrieve privat data 
    priv = container_of(inode->i_cdev, struct mcp2515_priv, mcp2515_cdev);
    if (!priv) {
        pr_err("Failed to retireve privat data from inode\n");
        return -EINVAL;
    }

    pr_info("Device closed\n");
    return ret;
}

static unsigned int mcp2515_poll(struct file *file, poll_table *wait)
{
    unsigned int mask = 0;
    struct mcp2515_priv *priv = file->private_data;

    // add the file to the wait queue 
    poll_wait(file, &priv->rx_wait_queue, wait);
    poll_wait(file, &priv->tx_wait_queue, wait);
    
    // check if data is available for reading 
    if (!circular_buffer_is_empty(&priv->rx_cbuf)) {
        mask |= (POLLIN | POLLRDNORM); 
    }

    // check if space is available for writing 
    if (!circular_buffer_is_full(&priv->tx_cbuf)) {
        mask |= (POLLOUT | POLLWRNORM);
    }

    return mask;
}

static ssize_t mcp2515_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    struct mcp2515_priv *priv = file->private_data;
    struct spi_device *spi = NULL;
    struct can_frame_data frame;

    // retrieve private data
    if (!priv) {
        pr_err("Failed to retrieve private data\n");
        return -EINVAL;
    }
    spi = priv->spi;

    // check for full frame size
    if (count != sizeof(frame)) {
        dev_err(&spi->dev, "Incorrect frame size\n");
        return -EINVAL;
    }

    // check if the file is opened in non-blocking mode
    if (file->f_flags & O_NONBLOCK) {
        // try to read the CAN message from the circular buffer
        ret = circular_buffer_read(&priv->rx_cbuf, &frame);
        if (ret) {
            return -EAGAIN; // no data available
        }
    } else {
        dev_err(&spi->dev, "Reading in Blocking-Mode is not supported yet\n");
        return -EINVAL;
    }

    // copy CAN message to the user space 
    ret = copy_to_user(buf, &frame, sizeof(frame));
    if (ret) {
        dev_err(&spi->dev, "Failed to copy CAN message to the user space\n");
        return -EFAULT;
    }

    return sizeof(frame);
}

static ssize_t mcp2515_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    struct mcp2515_priv *priv = file->private_data;
    struct spi_device *spi = NULL;
    struct can_frame_data frame;

    // retrieve private data
    if (!priv) {
        pr_err("Failed to retrieve private data\n");
        return -EINVAL;
    }
    spi = priv->spi;

    // check for full frame size
    if (count != sizeof(frame)) {
        dev_err(&spi->dev, "Incorrect frame size\n");
        return -EINVAL;
    }

    // copy data from user space to kernel space
    ret = copy_from_user(&frame, buf, count);
    if (ret) {
        dev_err(&spi->dev, "Failed to copy %zu bytes from user space\n", count - ret);
        return -EFAULT;
    }

    // check if the file is opened in non-blocking mode
    if (file->f_flags & O_NONBLOCK) {
        // write CAN Message to the transmit circular buffer (override accepted)
        ret = circular_buffer_write(&priv->tx_cbuf, &frame);
        if (ret) {
            // just signal ring buffer overflow (because it is valid condtion)
            dev_info(&spi->dev, "Transmit buffer overflow: tail message was overriden\n");
        }
    } else {
        dev_err(&spi->dev, "Writing in Blocking-Mode is not supported yet\n");
        return -EINVAL;
    }

    // schedule the transmit work queue
    queue_work(priv->tx_work_queue, &priv->tx_work);

    return count;
}

static long mcp2515_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long  ret = 0;
    struct mcp2515_priv *priv = file->private_data;
    struct spi_device *spi = NULL;

    // retrieve private data
    if (!priv) {
        pr_err("Failed to retrieve private data\n");
        return -EINVAL;
    }
    spi = priv->spi;

    switch(cmd) {
        case MCP2515_RESET_IOCTL: {
            dev_info(&spi->dev, "MCP2515 reset command received\n");
            
            // reset the hardware and block the caller thread until the functions returns
            ret |= mcp2515_reset_hw(spi);
            ret |= mcp2515_config_hw(spi);
            ret |= circular_buffer_reset(&priv->rx_cbuf);
            ret |= circular_buffer_reset(&priv->tx_cbuf);
            if (ret) {
                dev_err(&spi->dev, "Failed to reset the mcp2515 hardware\n");
                return ret;
            }

            dev_info(&spi->dev, "MCP2515 succesfully reseted\n");
            break;
        }
        case MCP2515_SET_RXBF_IOCTL: {
            struct rx_filter filter;
            u8 prev_opmode = 0x0;
            dev_info(&spi->dev, "MCP2515 set RXB filter command received\n");

            // retrieve filter data from user space 
            ret = copy_from_user(&filter, (struct rx_filter __user *)arg, sizeof(struct rx_filter));
            if (ret) {
                dev_err(&spi->dev, "Failed to copy rx_filter struct from user space\n");
                return -EFAULT;
            }

            // save the current operation mode
            ret |= mcp2515_get_opmode(spi, &prev_opmode);
            
            // switch the chip into the configuration mode 
            ret |= mcp2515_set_opmode(spi, MCP2515_CONFIG_MODE);

            // set the filter for receive buffer
            ret |= mcp2515_set_rxbf(spi, filter);

            // restore the old operation mode;
            ret |= mcp2515_set_opmode(spi, prev_opmode);
            
            // check the overall error status 
            if (ret) {
                dev_err(&spi->dev, "Failed to set RXB filter: %d", filter.number);
                return ret;
            }

            dev_info(&spi->dev, "MCP2515 RXB filter %d set to value: %d\n", filter.number, filter.value);
            break;
        }
        case MCP2515_GET_RXBF_IOCTL: {
            struct rx_filter filter;
            u8 prev_opmode = 0x0;
            dev_info(&spi->dev, "MCP2515 get RXB filter command received\n");

            // retireve filter number to read
            ret = copy_from_user(&filter, (struct rx_filter __user *)arg, sizeof(struct rx_filter));
            if (ret) {
                dev_err(&spi->dev, "Failed to copy rx_filter struct from user space\n");
                return -EFAULT;
            }

            // save the current operation mode
            ret |= mcp2515_get_opmode(spi, &prev_opmode);
            
            // switch the chip into the configuration mode 
            ret |= mcp2515_set_opmode(spi, MCP2515_CONFIG_MODE);

            // read the register value for the given filter number
            ret |= mcp2515_get_rxbf(spi, &filter);

            // restore the old operation mode;
            ret |= mcp2515_set_opmode(spi, prev_opmode);

            // check the overall error status 
            if (ret) {
                dev_err(&spi->dev, "Failed to get RX filter: %d\n", filter.number);
                return ret;
            }

            // write the filter data back to the user space 
            ret = copy_to_user((struct rx_filter __user *)arg, &filter, sizeof(struct rx_filter));
            if (ret) {
                dev_err(&spi->dev, "Failed to copy opmode-value to user space\n");
                return -EFAULT;
            }

            break;
        }
        case MCP2515_SET_OPMODE_IOCTL: {
            // retrieve the operation mode from user space argument 
            ret = copy_from_user(&priv->des_opmode, (u8 __user *)arg, sizeof(priv->des_opmode));
            if (ret) {
                dev_err(&spi->dev, "Failed to copy opmode-value from user space\n");
                return -EFAULT;
            }
            dev_info(&spi->dev, "MCP2515 set operation mode command received. Mode: %u\n", priv->des_opmode);

            // set the desired mode and block the caller thread until the functions returns
            ret = (long)mcp2515_set_opmode(spi, priv->des_opmode);
            if (ret) {
                return -EFAULT;
            }
            break;
        }
        case MCP2515_GET_OPMODE_IOCTL: {
            u8 opmode = 0;
            dev_info(&spi->dev, "MCP2515 get operation mode command received\n");

            // retrieve the current operation mode form the chip 
            ret = mcp2515_get_opmode(spi, &opmode);
            if (ret) {
                dev_err(&spi->dev, "Failed to retrieve the operation mode from the chip\n");
                return -EFAULT;
            } 

            // copy the operation mode value to user space 
            ret = copy_to_user((u8 __user *)arg, &opmode, sizeof(opmode));
            if (ret) {
                dev_err(&spi->dev, "Failed to copy opmode-value to user space\n");
                return -EFAULT;
            }
            break;
        }
        default:    
            return -ENOTTY; // command not supported 
    }
    return ret;
}

// Sets the mask value for the appropriate RX buffer (0 or 1)
static int mcp2515_set_rxbm(struct spi_device *spi, struct rx_mask mask)
{
    int ret = 0;
    int sidh_reg = 0x0;
    int sidl_reg = 0x0;

    // Determine the high register address based on mask number
    if (mask.number == 0)
        sidh_reg = MCP2515_RXM0SIDH;    // High register address for mask 0 (RX buffer 0)
    else if (mask.number == 1)
        sidh_reg = MCP2515_RXM1SIDH;    // High register address for mask 1 (RX buffer 1)
    else {
        dev_err(&spi->dev, "Error, mask number %d is not supported\n", mask.number);
        return -EINVAL; // Invalid argument error
    }
    
    sidl_reg = sidh_reg + 1; // Low register address is one byte after high register

    // Write to the mask registers 
    ret |= mcp2515_write_reg(spi, sidh_reg, ((mask.value >> 3) & 0xFF)); 
    ret |= mcp2515_write_reg(spi, sidl_reg, ((mask.value << MCP2515_RXM_SID_O) & MCP2515_RXM_SID_M)); 
    if (ret) {
        dev_err(&spi->dev, "Failed to set RX-Mask %d\n", mask.number);
    }

    return ret;
}

// Sets the filter value for the appropriate RX buffer (0 or 1)
static int mcp2515_set_rxbf(struct spi_device *spi, struct rx_filter filter)
{
    int ret = 0;
    u8 sidh_reg = 0x0;
    u8 sidl_reg = 0x0;

    // Validate the filter number
    if (filter.number < 0 || filter.number > 5) {
        dev_err(&spi->dev, "Error, filter number %d is not supported\n", filter.number);
        return -EINVAL; // Invalid argument error
    }
    
    // Adjust filter number for register calculation
    if (filter.number >= 3)
        filter.number++;
    
    // Calculate register addresses
    sidh_reg = filter.number * 4; // High register address
    sidl_reg = sidh_reg + 1;      // Low register address

    // Write to the filter registers 
    ret |= mcp2515_write_reg(spi, sidh_reg, ((filter.value >> 3) & 0xFF));  
    ret |= mcp2515_write_reg(spi, sidl_reg, ((filter.value << MCP2515_RXF_SID_O) & MCP2515_RXF_SID_M)); 
    if (ret) {
        dev_err(&spi->dev, "Failed to set RX-Filter %d\n", filter.number);
    }

    return ret;
}

// Reads the filter value for the appropriate RX buffer (0 or 1)
static int mcp2515_get_rxbf(struct spi_device *spi, struct rx_filter *filter)
{
    int ret = 0;
    u8 sidh_reg = 0x0;
    u8 sidl_reg = 0x0;
    u8 sidh_val = 0, sidl_val = 0;
    u8 adjusted_number = 0;

    // Validate the filter number
    if (filter->number < 2 || filter->number > 5) {
        dev_err(&spi->dev, "Error, filter number %d is not supported\n", filter->number);
        return -EINVAL; // Invalid argument error
    }
    
    // Adjust filter number for register calculation
    adjusted_number = filter->number;
    if (adjusted_number >= 3)
        adjusted_number++;
    
    // Calculate register addresses
    sidh_reg = adjusted_number * 4; // High register address
    sidl_reg = sidh_reg + 1;       // Low register address

    // Read filter registers
    ret |= mcp2515_read_reg(spi, sidh_reg, &sidh_val);  
    ret |= mcp2515_read_reg(spi, sidl_reg, &sidl_val); 
    if (ret) {
        dev_err(&spi->dev, "Failed to read register for RX-Filter %d\n", filter->number);
        return ret;
    }

    // Combine the high and low byte values into the filter value
    filter->value = ((sidh_val << 3) | ((sidl_val & MCP2515_RXF_SID_M) >> MCP2515_RXF_SID_O));

    pr_info("sidh: %u, sidl: %u for filter number: %d", sidh_val, sidl_val, filter->number);

    return 0;
}


// Probing the device
static int mcp2515_spi_probe(struct spi_device *spi)
{
    int ret = 0;      // return value on success 
    u32 nofTq = 1;    // number of time quants used in the baudrate calculation
    u32 temp_u32 = 0; // temporary variable for reading device tree values

    u8 canstat = 0;   // can-status register value 

    struct device_node *np = spi->dev.of_node;
    struct mcp2515_platform_data *pdata;
    struct mcp2515_priv *priv;
    struct device *dev_node;

    // allocate memory for platform data (automatically freed by device management system)
    pdata = devm_kzalloc(&spi->dev, sizeof(struct mcp2515_platform_data), GFP_KERNEL);
    if (!pdata) {
        dev_err(&spi->dev, "Failed to allocate memory for platform data\n");
        return -ENOMEM;
    }

    // allocate memory for privat data (automatically freed by device management system)
    priv = devm_kzalloc(&spi->dev, sizeof(struct mcp2515_priv), GFP_KERNEL);
    if (!priv) {
        dev_err(&spi->dev, "Failed to allocate memory for privat data\n");
        return -ENOMEM;
    }

    // associate priv data with spi_device 
    spi_set_drvdata(spi, priv); // set drivers private data
    priv->pdata = pdata; // store platform data in privat data
    priv->spi = spi;     // store spi pointer in privat data 

    // parse the interrupt from the device tree
    pdata->irq = of_irq_get(np, 0);
    if (pdata->irq < 0) {
        dev_err(&spi->dev, "Failed to get IRQ from device tree\n");
        return pdata->irq;
    }

    // read CAN-Bus parameters from the device tree 
    if (of_property_read_u32(np, "oscillator-frequency", &pdata->osc_freq) ||
        of_property_read_u32(np, "ost", &pdata->ost) ||
        of_property_read_u32(np, "brp", &temp_u32) || (pdata->brp = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "tsync", &temp_u32) || (pdata->tsync = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "tprop_seg", &temp_u32) || (pdata->tprop_seg = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "ps1", &temp_u32) || (pdata->ps1 = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "ps2", &temp_u32) || (pdata->ps2 = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "sjw", &temp_u32) || (pdata->sjw = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "btlmode", &temp_u32) || (pdata->btlmode = (u8)temp_u32, 0) ||
        of_property_read_u32(np, "sam", &temp_u32) || (pdata->sam = (u8)temp_u32, 0)) {
            dev_err(&spi->dev, "Failed to read config. parameters from device tree\n");
            return -EINVAL;
    }

    // set the delay interval and timeout itme for the waiting queue
    priv->opmode_work_data.delay_ms = 10;  // this value is mutch higher than the necessery 16us with ost=128 and f_osc=8MHz
    priv->opmode_work_data.timeout_ms = 100 * priv->opmode_work_data.delay_ms;    // timeout after 1s 
 
    // calculate and print the baudrate of the CAN-Bus (just for the test purpose)
    nofTq = pdata->tsync + pdata->tprop_seg + pdata->ps1 + pdata->ps2;
    pdata->baudrate = pdata->osc_freq/(2 * pdata->brp * nofTq);
    dev_info(&spi->dev, "Calculated baudrate: %dbps\n", pdata->baudrate);

    // request IRQ with the top half (ISR) and bottom half (threaded handler)
    ret = devm_request_threaded_irq(&spi->dev, spi->irq, mcp2515_irq, mcp2515_irq_handler, 0, "mcp251%_irq", priv);
    if (ret) {
        dev_err(&spi->dev, "Failed to request IRQ %d: %d", spi->irq, ret);
        return ret;
    }

    // initialize the spinlock
    spin_lock_init(&priv->lock); // need ? -> use atomic var instead 

    // initialize spi mutex lock 
    mutex_init(&priv->spi_lock);

    // initialize operation mode setup wait queue  
    init_waitqueue_head(&priv->opmode_wait_queue);

    // create a workqueue for operation mode setup time 
    priv->opmode_work_queue = create_singlethread_workqueue("mcp2515_opmode_work_queue");
    if (!priv->opmode_work_queue) {
        dev_err(&spi->dev, "Failed to create operation mode setup work queue\n");
        return -ENOMEM;
    }

     // initialize receive and transmit circular buffers
    circular_buffer_init(&priv->rx_cbuf);
    circular_buffer_init(&priv->tx_cbuf);

    // initialize TXn and Rxn synchronization wait queues
    init_waitqueue_head(&priv->rx_wait_queue);
    init_waitqueue_head(&priv->tx_wait_queue);

    // create a workqueue for transmit work
    priv->tx_work_queue = create_singlethread_workqueue("mcp2515_tx_work_queue");
    if (!priv->tx_work_queue) {
        dev_err(&spi->dev, "Failed to create transmit work queue\n");
        return -ENOMEM;
    }

    // initialize the transmit work structure
    INIT_WORK(&priv->tx_work, mcp2515_write_can_frame_work);

    // initialize the atomic varaible to signal that the MCP2515 TX2 buffer is ready for new data
    atomic_set(&priv->tx_buf_ready, 1);   // at the begin the TX2 buffer is empty and thus ready for new data  

    // configure the chip hardware 
    
    // reset the hardware and enter configuration mode
    ret = mcp2515_reset_hw(spi); 
    if (ret) {
        dev_err(&spi->dev, "Failed to reset the hardware\n");
        return ret;
    }

    // configure the hardware 
    ret = mcp2515_config_hw(spi);  
    if (ret) {
        dev_err(&spi->dev, "Failed to configure the mcp2515 hardware\n");
        return ret;
    }

    // read CAN-status register of debuging purpose:
    ret = mcp2515_read_reg(spi, MCP2515_CANSTAT, &canstat);
    if (ret) {
        dev_err(&spi->dev, "Failed to read status register\n");
        return ret;
    }
    dev_info(&spi->dev, "CANSTAT: 0x%02x\n", canstat);

    /* create character device node to expose communication interface to user space: */
    // request character device identifier
    ret = alloc_chrdev_region(&priv->dev_num, 0, 1, MCP2515_DEVICE_NAME);
    if (ret) {  
        dev_err(&spi->dev, "Failed to request character device identifier\n");
        return ret;
    }
   
    // initialize the cdev and bind it to the file operations
    cdev_init(&priv->mcp2515_cdev, &mcp2515_fops);
    priv->mcp2515_cdev.owner = THIS_MODULE;

    // register the cdev with the kernel and make it available to the user space
    ret = cdev_add(&priv->mcp2515_cdev, priv->dev_num, 1);
    if (ret) {
        dev_err(&spi->dev, "Failed to register cdev with the kernel\n");
        goto failed_cdev_add;
    }

    // create device class, visible in /sys/class
    priv->mcp2515_class = class_create(THIS_MODULE, MCP2515_CLASS);
    if (IS_ERR(priv->mcp2515_class)) {
        dev_err(&spi->dev, "Failed to create device class\n");
        ret = PTR_ERR(priv->mcp2515_class);
        goto failed_class_create;
    }

    // create device node in /dev and register with sysfs 
    dev_node = device_create(priv->mcp2515_class, NULL, priv->dev_num, priv, MCP2515_DEVICE_NAME);
    if (IS_ERR(dev_node)) {
        dev_err(&spi->dev, "Failed to create device node\n");
        ret = PTR_ERR(dev_node);
        goto failed_device_create;
    }

    dev_info(&spi->dev, "Device probed successfully\n"); 

    return ret;

// device registration error labels
failed_device_create:
    class_destroy(priv->mcp2515_class);
failed_class_create: 
    cdev_del(&priv->mcp2515_cdev);
failed_cdev_add:
    unregister_chrdev_region(priv->dev_num, 1);
	return ret;
}

// removing the device 
static void mcp2515_spi_remove(struct spi_device *spi)
{
    struct mcp2515_priv *priv = spi_get_drvdata(spi);

    if (!priv) {
        dev_err(&spi->dev, "Failed to get privat data\n");
        return;
    }

    // destroy character device and remove it from the system 
    device_destroy(priv->mcp2515_class, priv->dev_num);
    class_destroy(priv->mcp2515_class); 
    cdev_del(&priv->mcp2515_cdev);
    unregister_chrdev_region(priv->dev_num, 1);

    // cancel and destroy operation mode workqueue
    if (priv->opmode_work_queue) {
        cancel_delayed_work_sync(&priv->opmode_work_data.work);
        destroy_workqueue(priv->opmode_work_queue);
    }

    // cancel and destroy the transmit work queue
    if (priv->tx_work_queue) {
        cancel_work_sync(&priv->tx_work);       // cancel ongoing work
        destroy_workqueue(priv->tx_work_queue); // destroy the work queue 
    }

    // ensure no processes are left waiting on the wait queues
    wake_up_all(&priv->opmode_wait_queue);
    wake_up_all(&priv->rx_wait_queue);
    wake_up_all(&priv->tx_wait_queue);

	dev_info(&spi->dev, "Driver removed successfully\n");
}

/* lists specific devices the driver supports
 * ensures the right driver is loaded for the connected SPI device
 * facilitates automatic device initialisation and driver binding */
static const struct spi_device_id mcp2515_spi_id[] = {
	{ "mcp2515", 0 },	// entry for the MCP2515 device
	{ },	// termination
};
MODULE_DEVICE_TABLE(spi, mcp2515_spi_id);

/* structure used in conjunction with the Device Tree
 * to match device tree nodes with driver instances */
static const struct of_device_id mcp2515_of_match[] = {
	{ .compatible = "microchip,mcp2515", },
	{},		// termination
};
MODULE_DEVICE_TABLE(of, mcp2515_of_match);

// provisioning the device in the driver 
static struct spi_driver mcp_spi_driver = {
	.driver = {
		.name = "mcp2515",		// driver name
		.owner = THIS_MODULE,	// module ownership 
		.of_match_table = of_match_ptr(mcp2515_of_match),	// match device tree nodes with driver instances
	},
	.probe = mcp2515_spi_probe,		// probe function
	.remove = mcp2515_spi_remove,	// remove function 
	.id_table = mcp2515_spi_id,		// Device-ID table
};	

// module initialization 
static int __init mcp2515_module_init(void)
{
	int ret = 0;

	ret = spi_register_driver(&mcp_spi_driver);	// register the SPI driver
	if (ret)
		printk(KERN_ERR "Failed to register SPI driver: %d\n", ret);
	else
		printk(KERN_INFO "SPI driver registered successfully\n");

	return ret; 
}

// module exit 
static void __exit mcp2515_module_exit(void)
{
	spi_unregister_driver(&mcp_spi_driver);
	printk(KERN_INFO "SPI driver unregistered\n");

}

module_init(mcp2515_module_init);
module_exit(mcp2515_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andreas Cnaus");
MODULE_DESCRIPTION("MCP2515 SPI Driver");

