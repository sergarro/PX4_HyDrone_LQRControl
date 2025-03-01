#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/fuelcellbatt.h>



__EXPORT int prueba2_main(int argc, char *argv[]);

int prueba2_main(int argc, char *argv[])
{
    struct fuelcellbatt_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(fuelcellbatt), &att);

    att.timestamp=666;
    att.timestamp_sample=666;
    att.pila=666;
    att.stack_voltage=666;
    att.load_currente=666;
    att.power=666;
    att.energy=666;
    att.battery_voltage=666;
    att.battery_current=666;
    att.load_voltage=666;
    att.temp1=666;
    att.temp2=666;
    att.temp3=666;
    att.temp4=666;
    att.target_temp=666;
    att.board_temp=666;
    att.h2_pressure=666;
    att.fan_speed=666;
    att.state=666;
    att.signo=666;
    att.top_temp=666;

    orb_publish(ORB_ID(fuelcellbatt), att_pub, &att);
    return 0;
}
