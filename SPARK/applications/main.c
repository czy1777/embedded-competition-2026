#include <bsp_system.h>

int main(void)
{
    rt_kprintf("Main thread running...\n");

    while (1)
    {
        rt_thread_mdelay(1000);
    }

    return 0;
}
