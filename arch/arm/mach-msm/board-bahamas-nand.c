#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>

int __init unprotect_nand(void)
{
	int* addr;
	printk("Hack: unprotecting Tattoo system partition\n");
	addr = ioremap(0xA0B00000, 0x1000);
	if (addr) {
		printk("NAND protect value 0x%X\n", *addr);
		*addr = 0;
		iounmap(addr);
	}
	printk("Done - now be extremly careful!!!\n");
	return 0;
}

void __exit unprotect_nand_exit(void)
{
	printk("Unprotect NAND module exit!\n");
}

module_init(unprotect_nand);
module_exit(unprotect_nand_exit);
MODULE_DESCRIPTION("Tattoo hack - disable write protect");
MODULE_AUTHOR("bool_s");
MODULE_LICENSE("GPL");
