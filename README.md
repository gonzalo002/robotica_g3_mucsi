# Robótica MUCSI Grupo 7
## Instalación

### 1. Clonar este repositorio

Clona este repositorio en tu máquina local:

```bash
git clone https://github.com/gonzalo002/robotica_g7_mucsi
```
```bash
cd robotica_g7_mucsi
```

### 2. Construir la imagen de los contenedores
Actualmente solo esta preparado para local_gpu, por lo que se solo tienes una opción.

Una vez elegido el contenedor ejecutar:
```bash
docker compose --profile grupo_7_local_gpu build
```
Cuando acabe el proceso ya se dispondrá de la imagen construida en el sistema.
### 3. Lanzar el contenedor
Una vez construida la imagen es posible crear contenedores a partir de ella. En este caso los contenedores están diseñados para utilizarlos como entornos de desarrollo. Se recomienda crear y utilizar un solo contenedor. Para esto ejecutar:
```bash
docker compose --profile grupo_7_local_gpu up -d
```
Una vez lanzado, la terminal se mantendrá ejecutando el contenedor hasta que se pare. Para pararlo basta con enviar una señal de terminación `ctrl`+`c`.

Es posible y recomendable gestionar el lanzamiento y parada de los contenedores desde la extensión remote de VSCode. Para gestionar la parada y lanzamiento de los contenedores ya creados desde VSCode:

### 4. Acceso al contenedor
- Para todos los contenedores se recomienda asociar una instancia de VSCode al contenedor utilizando la extension "remote explorer". Esto permite desarrollar en VSCode como si se trabajara en local, pero ejecutando todo en el contenedor.

### 5. Primeros pasos en ROS
Dentro de los contenedores se incluye un espacio de trabajo de ROS con todos los elementos necesarios para poder trabajar con los robots del laboratorio.

Antes de poder utilizarlos es necesario construir el espacio de trabajo base. Para construir el espacio de trabajo:

1. Conectarse al contenedor desde una instancia de VSCode.
2. Abrir una nueva terminal en VSCode.
3. Navegar hasta el directorio base del espacio de trabajo:
```bash
cd /home/laboratorio/ros_workspace
```
4. Actualizar la lista de paquetes disponibles del sistema:
```bash
sudo apt update
```
5. Actualizar el gestor de paquetes de ROS:
```bash
rosdep update
```
6. Instalar todas las dependencias del espacio de trabajo base:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
7. Construir el espacio de trabajo:
```bash
catkin build
```
Si todo se ha ejecutado de manera correcta, el espacio de trabajo ya está en condiciones de uso.

Antes de poder utilizar el espacio de trabajo hay que activarlo en la terminal:
```bash
source /home/laboratorio/ros_workspace/devel/setup.bash
```
>**IMPORTANTE:**
**<u>La activación del espacio de trabajo se debe realizar en cada nueva terminal abierta en la que se quiera utilizar algo relacionado con este.</u>**

Si siempre se va a utilizar el mismo espacio de trabajo, es posible incluir la activación en el fichero `/home/laboratorio/.bashrc` que se ejecuta cada vez que se abre una nueva terminal. Si se incluye, no hace falta volver a activar el espacio de trabajo.

Para comprobar que el sistema funciona correctamente, ejecutar en la terminal donde se ha activado el espacio de trabajo:
```bash
roslaunch launcher_robots_lab_robotica sim_203.launch
```

## Subir Archivos al Repositorio
### 1. Clonar el repositorio

```bash
git clone https://github.com/gonzalo002/robotica_g7_mucsi
```

### 2. Crear una nueva rama

```bash
cd cd robotica_g7_mucsi
```

```bash
git checkout -b <Nombre de la rama>
```

### 3. Añade los documentos que quieras importar en el repositorio clonado en tus archivos locales

Aqui simplemente con meter los documentos que quieras en las carpetas locales sin modificar lo que ya existe vale.

### 5. Agregar archivos al indice de Git

```bash
git add .
```

### 6. Realizar un commit de los cambios realizados

```bash
git commit -m "Mensaje descriptivo de lo que se cambia"
```

### 7. Realizar un push al repositorio original

```bash
git push origin <Nombre de la rama>
```
