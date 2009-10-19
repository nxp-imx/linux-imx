#
# Makefile for the i2c slave.
#

i2c_slave-objs := i2c_slave_ring_buffer.o i2c_slave_device.o i2c_slave_core.o
obj-$(CONFIG_I2C_SLAVE_CORE)	+= i2c_slave.o
obj-$(CONFIG_MXC_I2C_SLAVE)		+= mxc_i2c_slave.o
obj-$(CONFIG_I2C_SLAVE_CLIENT)		+= i2c_slave_client.o
