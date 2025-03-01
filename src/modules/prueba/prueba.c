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


#define SERIAL_PORT "/dev/ttyS6"
#define BUFFER_SIZE 145

__EXPORT int prueba_main(int argc, char *argv[]);

int prueba_main(int argc, char *argv[])
{

    PX4_INFO("Iniciando puerto...");

    int serialPort;
    struct termios serialSettings;
    char buffer[BUFFER_SIZE];
    int buffIndex = 0;

    // Abrir el puerto serie en modo lectura
    serialPort = open(SERIAL_PORT, O_RDONLY);
    if (serialPort == -1) {
        perror("Error al abrir el puerto serie");
        return 1;
    }

    // Configurar la velocidad y otros parámetros del puerto serie
    tcgetattr(serialPort, &serialSettings);
    cfsetispeed(&serialSettings, B19200);  // Velocidad de baudios
    serialSettings.c_cflag &= ~PARENB;    // Sin paridad
    serialSettings.c_cflag &= ~CSTOPB;    // 1 bit de parada
    serialSettings.c_cflag &= ~CSIZE;     // Limpiar bits de tamaño de carácter
    serialSettings.c_cflag |= CS8;        // 8 bits de datos
    tcsetattr(serialPort, TCSANOW, &serialSettings);

    /* advertise attitude topic */
	struct fuelcellbatt_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(fuelcellbatt), &att);

    // Leer del puerto serie y almacenar en el vector buff
    while (1) {

        char receivedChar;
        ssize_t bytesRead = read(serialPort, &receivedChar, sizeof(receivedChar));
        if (bytesRead > 0) {

            if (receivedChar == '\n') {
                buffer[buffIndex] = '\0';  // Agregar terminador de cadena
                printf("Lectura: %s\n", buffer);
                if (buffIndex >= 100) { //este if es lo nuevo borrar si deja de funcionar

                    //buffer[buffIndex] = '\0';  // Agregar terminador de cadena
                    //printf("Lectura (tamaño máximo alcanzado): %s\n", buffer);

                    char datos[55];
                    int contador = 0;
                    int caso=1;
                    for (int j = 0; j < (buffIndex+2); j++) {
                        if (isdigit(buffer[j])) {
                            datos[contador++] = buffer[j];
                        }
                        if(buffer[j] == ','){
                            datos[contador] = '\0';
                            switch (caso) {
                                case 1:
                                    printf("%s,",datos);
                                    att.stack_voltage=atof(datos);
                                    break;
                                case 2:
                                    printf("%s,",datos);
                                    att.load_currente=atof(datos);
                                    break;
                                case 3:
                                    printf("%s,",datos);
                                    att.power=atof(datos);
                                    break;
                                case 4:
                                    printf("%s,",datos);
                                    att.energy=atof(datos);
                                    break;
                                case 5:
                                    printf("%s,",datos);
                                    att.battery_voltage=atof(datos);
                                    break;
                                case 6:
                                    printf("%s,",datos);
                                    att.battery_current=atof(datos);
                                    if(buffer[j-5]=='-'){
                                        att.signo=1;
                                    }
                                    else{
                                        att.signo=0;
                                    }
                                    break;
                                case 7:
                                    printf("%s,",datos);
                                    att.load_voltage=atof(datos);
                                    break;
                                case 8:
                                    printf("%s,",datos);
                                    att.temp1=atof(datos);
                                    int temp1 = atof(datos);
                                    break;
                                case 9:
                                    printf("%s,",datos);
                                    att.temp2=atof(datos);
                                    int temp2 = atof(datos);
                                    break;
                                case 10:
                                    printf("%s,",datos);
                                    att.temp3=atof(datos);
                                    int temp3 = atof(datos);
                                    break;
                                case 11:
                                    printf("%s,",datos);
                                    att.temp4=atof(datos);
                                    int temp4 = atof(datos);
                                    int max = fmax(fmax(temp1, temp2), fmax(temp3, temp4));
                                    att.top_temp=max;
                                    break;
                                case 12:
                                    printf("%s,",datos);
                                    att.target_temp=atof(datos);
                                    break;
                                case 13:
                                    printf("%s,",datos);
                                    att.board_temp=atof(datos);
                                    break;
                                case 14:
                                    printf("%s,",datos);
                                    att.pila=atof(datos);
                                    break;
                                case 15:
                                    printf("%s,",datos);
                                    att.h2_pressure=atof(datos);
                                    break;
                                case 16:
                                    printf("%s,",datos);
                                    att.fan_speed=atof(datos);
                                    break;
                                case 17:
                                    printf("%s,\n",datos);
                                    att.state=atof(datos);
                                    break;
                                default:
                                    printf("Opción inválida\n");
                            }
                            caso++;
                            memset(datos, 0, 55);
                            contador=0;
                        }
                    }
                    orb_publish(ORB_ID(fuelcellbatt), att_pub, &att);
                }
                buffIndex = 0;  // Reiniciar el índice del vector buff
            }
            else {
                buffer[buffIndex] = receivedChar;
                buffIndex++;
            }
        }
    }

    PX4_INFO("Fuera del while");
    // Cerrar el puerto serie
    close(serialPort);
    return 0;
}
