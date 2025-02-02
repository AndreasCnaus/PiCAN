#!/bin/bash

sudo rmmod mcp2515
sudo insmod build/mcp2515.ko
sudo chmod 666 /dev/mcp2515