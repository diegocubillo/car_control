# Instrucciones para añadir los mensajes ROS2 personalizados a MATLAB

Para añadir mensajes ROS2 personalizados presentes en el submódulo tcp_ros_packages a MATLAB, sigue estos pasos:

1. **Generar los archivos de mensajes ROS2 en MATLAB:**
  - Usa el comando `ros2genmsg` para generar los archivos de mensajes. Para ello, introduce el path absoluto hasta el paquete ros. Por ejemplo:
    ```matlab
    folderPath = '~/car_control/external/tcp_ros_packages';
    ros2genmsg(folderPath);
    ```

2. **Añadir los mensajes generados al entorno de MATLAB:**
  - Después de generar los archivos de mensajes, añade la carpeta generada al path de MATLAB usando el comando `addpath`. Por ejemplo:
    ```matlab
    addpath('~/car_control/external/tcp_ros_packages/matlab_msg_gen');');
    ```

3. **Verificar la instalación de los mensajes:**
  - Para asegurarte de que los mensajes se han añadido correctamente, puedes listar los tipos de mensajes disponibles usando el comando `ros2 msg list`. Por ejemplo:
    ```matlab
    ros2 msg list
    ```

Recuerda reemplazar `~/` con la ruta real al directorio donde tengas descargado car_control.

## Compilación de nodo ROS2 en Raspberry Pi 4
1. **Abrir el modelo de simulink con su configuración:**
  - Ejecutar config_car.m con RUN_MODE=5. El modelo de simulink se abrirá ya configurado para cargar el nodo ROS2.

2. **Configurar los datos de la Raspberry Pi 4:**
  - Introducir en ROS -> CONNECT -> Deploy to -> Manage Remote Device la IP, usuario, contraseña y directorios solicitados para el envío al hardware remoto del nodo. Mediante el botón Test Connection se puede verificar que haya conexión, todos los datos sean correctos y matlab tenga acceso al hardware.

3. **Compilar el nodo ROS2 en moso externo:**
  - Hacer clic en ROS -> DEPLOY -> Build Model y esperar a que se compile el nodo. Este proceso puede tardar unos 15 minutos.