import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt

def _draw_pyramid_from_matrices(plant_matrix, side_matrix, top_matrix):
    fig_3d = plt.figure(figsize=(12, 8), dpi=100)

    # Crear tres subgráficos para alzado, planta y lateral
    ax_plant = fig_3d.add_subplot(131, projection='3d')  # Planta
    ax_side = fig_3d.add_subplot(132, projection='3d')   # Lateral (frontal)
    ax_top = fig_3d.add_subplot(133, projection='3d')    # Alzado

    # Definir los colores de los cubos
    colors = {0: 'red', 1: 'green', 2: 'blue', 3: 'yellow'}

    # Definir el tamaño de cada cubo
    size = 1

    # Función para dibujar cubos en una vista 3D
    def draw_cubes(ax, matrix, is_side=False):
        for i in range(len(matrix)):  # Para cada fila
            for j in range(len(matrix[i])):  # Para cada columna
                if matrix[i][j] != -1:  # Si hay un cubo en la vista
                    height = -1
                    if is_side:  # Para la vista lateral, considerar altura
                        for k in range(len(side_matrix)):
                            if side_matrix[k][j] != -1 and side_matrix[k][j] == matrix[i][j]:
                                height = (len(side_matrix) * size) - 1 - k
                                break
                    else:
                        height = 0  # Alzado y planta tienen altura 0 por defecto

                    # Dibujar el cubo
                    if height != -1:
                        ax.bar3d(
                            j * size,  # X
                            (len(matrix) - 1 - i) * size,  # Y (invertido para ver arriba hacia abajo)
                            height * size,  # Z
                            size, size, size, 
                            color=colors[matrix[i][j]]
                        )

                    # Dibujar los cubos debajo si es necesario (solo en la vista lateral)
                    if is_side and height != -1:
                        for k in range(height):
                            ax.bar3d(
                                j * size, 
                                (len(matrix) - 1 - i) * size, 
                                k * size,  # Z de los cubos en gris
                                size, size, size, 
                                color='gray'
                            )

    # Dibujar cubos en las tres vistas
    draw_cubes(ax_plant, plant_matrix)  # Planta
    draw_cubes(ax_side, side_matrix, is_side=True)  # Lateral
    draw_cubes(ax_top, top_matrix)  # Alzado

    # Configurar límites y etiquetas de los ejes
    for ax in [ax_plant, ax_side, ax_top]:
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_box_aspect([1, 1, 1])  # Asegura una proporción equitativa en las tres vistas

    # Ajustar las vistas para cada subgráfico
    ax_plant.view_init(elev=90, azim=0)  # Planta
    ax_side.view_init(elev=30, azim=-60)  # Lateral
    ax_top.view_init(elev=0, azim=0)  # Alzado

    # Mostrar la figura
    plt.show()  # Muestra la figura con las tres vistas



if __name__ in "__main__":
    top_side = [[-1, -1, -1, -1, -1],
                  [-1, -1, -1, -1, -1],
                  [1, -1, 3, -1, -1],
                  [3, -1, 1, -1, -1],
                  [2, 3, 0, -1, -1]]

    front_side = [[-1, -1, -1, -1, -1],
                  [-1, -1, -1, -1, -1],
                  [2, -1, 0, -1, -1],
                  [1, -1, 3, -1, -1],
                  [0, 3, 2, -1, -1]]
    
    front_side_2 = [[-1, -1, -1, -1, -1],
                  [-1, -1, -1, -1, -1],
                  [1, -1, 2, -1, -1],
                  [3, 0, 1, -1, -1],
                  [1, 3, 0, -1, -1]]
    
    _draw_pyramid_from_matrices(top_side, front_side, front_side_2)
