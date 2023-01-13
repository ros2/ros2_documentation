# Contribución en un proyecto de código abierto

Es importante empezar por el documento llamado README, que como su nombre indica, conviene leer. Este fichero responderá tus preguntas más generales acerca del proyecto, como qué es, qué hace y cómo lo hace. También puedes acceder a la pestaña de issues. No es más que una lista de tareas y problemas que están pendientes y aún no han sido solucionadas.

Para trabajar en el proyecto, necesitarás una copia local del repositorio que contiene el código. Puedes hacer un fork desde Github y posteriormente clonarlo en tu computadora, por eso se describen una serie de pasos que pueden servir de guía a continuación:
# Paso 1: Fork del proyecto

Haz click en el botón **fork** en la parte superior izquierda

  

Luego de seleccionar el botón fork tendrás el repositorio en tu cuenta de GitHub.

# Paso 2: Clonar el proyecto

Debes copiar la ruta url y clonar el repositorio. Una url HTTPS tiene este formato https://github.com/user/repo.git

    git clone [https://github.com/ros2/ros2_documentation.git]

# Paso 3: Crea upstream

Esto es necesario para poder mantener el seguimiento desde el repositorio local al repositorio del proyecto original,debido a que algunos repositorios aceptan los pull request cada cierto tiempo, así que debes asegurarte que tu rama tenga los últimos cambios del repositorio original. Debes copiar la **url** de upstream.

Para crear un link al repositorio original, copia y pega el siguiente comando en tu terminal:

    git remote add upstream <upstream address>

> Nota: Para confirmar que no ha habido ningún cambio hasta este momento
> (desde que realizamos el fork del repositorio hasta ahora), puedes
> usar git pull upstream master

# Paso 4: Crea la rama dónde vas a trabajar

Esto nos permite identificar que la **rama** es para la contribución que se está a punto de hacer, podría ser desde corregir un error de tipografía hasta una traducción completa de una sección. De cualquier manera, es buena práctica crear una rama.

Utiliza nombres fáciles de leer y descriptivos, por ejemplo, debes crear una rama llamado 

*traduccion-ros2-nav*

Para crear una rama debes escribir el siguiente comando en tu terminal:

    git checkout -b <el-nombre-de-tu-rama>

Este comando creará una rama y apuntará a la rama maestra.

Añade tu contribución, desde tu computadora local y salvalos. Con este comando podrás ver los archivos modificados

    git status

# Paso 5: Git add y hacer commit de tu contribución

Puedes hacer stage y commit de tus cambios escribiendo lo siguiente en la terminal:

// Para hacer stage de los cambios

`git add .` ( si quieres añadir todos los archivos modificados)

//Para hacer commit de los cambios

    git commit -m 'Mensaje-del-commit'

> Notas: Link a buenas prácticas para escribir un commit
> https://midu.dev/buenas-practicas-escribir-commits-git/

# Paso 6: Hacer Pull desde upstream a nuestra rama local

Es mezclar cualquier diferencia en upstream en nuestra local para prevenir conflictos.

    git pull upstream <nombre-de-rama>

# Paso 7: Push de la rama en la que estamos trabajando

Para hacer push de los cambios en los que has estado trabajando debes ejecutar:

    git push origin <nombre-de-rama>

# Paso 8: Crear un pull request

Este es el paso final, donde es importante destacar los cambios realizados, recuerda hacer referencia a la issue de partida. Para abrir un pull request, navega hasta el repositorio principal como puedes ver en la siguiente imagen:

  

Podrás ver la última rama que subiste *'traduccion-ros2-nav'*, entonces debes hacer click en **'compare and pull request'.** Al abrir una solicitud de pull request, esperaras que el o los revisor o revisores del proyecto den el ok.

  
  

Listo!,gracias por tu contribución, una vez que sea aprobado vas a poder hacer Merge, desde el pull request creado, haciendo click en en el botón Merge