#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

// 1. ESTRUCTURA DE DATOS
typedef struct {
    double acc[3];
    double gyro[3];
    double roll;
    double pitch;
    double yaw;
    double temp;
    uint64_t timestamp;
} IMUData;

// 2. FUNCIÓN DE INICIALIZACIÓN
// Retorna el descriptor de archivo (fd) del puerto serial, o -1 si falla.
int init_imu(const char* port_name) {
    int serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        return -1;
    }

    struct termios tty;
    tcgetattr(serial_fd, &tty);
    cfsetospeed(&tty, B115200); 
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tcsetattr(serial_fd, TCSANOW, &tty);

    // Comandos de desbloqueo y reinicio
    unsigned char unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    unsigned char reset_yaw[]  = {0xFF, 0xAA, 0x01, 0x03, 0x00};
    
    write(serial_fd, unlock_cmd, 5);
    usleep(100000); // 100 ms
    write(serial_fd, reset_yaw, 5);
    usleep(500000); // 500 ms

    return serial_fd;
}

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

void FinishAttitude(boost::posix_time::ptime now, mjmech::mech::AttitudeData* attitude) {
    static int serial_fd = -1;
    
    // 1. APERTURA Y BAUD RATE ROBUSTOS
    if (serial_fd < 0) {
        serial_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd >= 0) {
            struct termios options;
            tcgetattr(serial_fd, &options);
            cfsetispeed(&options, B115200); // BAUD RATE de recepción
            cfsetospeed(&options, B115200); // BAUD RATE de transmisión
            
            // Configurar puerto en modo "crudo" (raw) para lectura de bytes puros
            options.c_cflag |= (CS8 | CREAD | CLOCAL);
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_oflag &= ~OPOST;
            tcsetattr(serial_fd, TCSANOW, &options);
        } else {
            return; // No se pudo abrir, intentará de nuevo en el siguiente ciclo
        }
    }

    // Usamos variables estáticas para mantener el estado entre los ciclos de 400Hz
    static unsigned char packet[4];
    static int p_index = 0;
    
    unsigned char current_byte;

    // 2. LECTURA COMPATIBLE CON ALTA FRECUENCIA (O_NONBLOCK)
    while (read(serial_fd, &current_byte, 1) > 0) {
        // Sincronización del header
        if (p_index == 0 && current_byte != 0x55) continue;
        
        packet[p_index++] = current_byte;

        // Si ya tenemos el paquete completo (11 bytes: 0x55, Type, 8 Datos, 1 Checksum)
        if (p_index == 11) {
            p_index = 0; // Reiniciar para el siguiente paquete
            
            unsigned char type = packet[5];
            int16_t v[6];
            for(int i = 0; i < 4; i++) {
                v[i] = (int16_t)((packet[3 + i*2] << 8) | packet[2 + i*2]);
            }

            // 3. ACTUALIZACIÓN DE LA ESTRUCTURA ATTITUDE
            if (type == 0x51) {
                attitude->accel_mps2.x() = (-1.0 * v[5] / 32768.0 * 16.0) * 9.80665; 
                attitude->accel_mps2.y() = (-1.0 * v / 32768.0 * 16.0) * 9.80665;
                attitude->accel_mps2.z() = (-1.0 * v[7] / 32768.0 * 16.0) * 9.80665;
            } 
            else if (type == 0x52) {
                attitude->rate_dps.x() = -1.0 * v[5] / 32768.0 * 2000.0;
                attitude->rate_dps.y() = -1.0 * v / 32768.0 * 2000.0;
                attitude->rate_dps.z() = -1.0 * v[7] / 32768.0 * 2000.0;
            } 
            else if (type == 0x53) {
                attitude->euler_deg.roll  = -1.0 * v[5] / 32768.0 * 180.0;
                attitude->euler_deg.pitch = -1.0 * v / 32768.0 * 180.0;
                attitude->euler_deg.yaw   = -1.0 * v[7] / 32768.0 * 180.0;
                
                // Construcción del Cuaternión (CRÍTICO PARA LA CINEMÁTICA)
                attitude->attitude = mjmech::base::Quaternion::FromEuler(
                    mjmech::base::Radians(attitude->euler_deg.roll),
                    mjmech::base::Radians(attitude->euler_deg.pitch),
                    mjmech::base::Radians(attitude->euler_deg.yaw)
                );
            }
        }
    }

    attitude->timestamp = now;
    attitude->bias_dps = mjmech::base::Point3D(0, 0, 0);
    attitude->bias_uncertainty_dps = mjmech::base::Point3D(0, 0, 0);
    attitude->attitude_uncertainty = mjmech::base::Quaternion(1.0, 0.0, 0.0, 0.0);
}