import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt

def find_first_non_minus_one(matrix):
    # Iteramos sobre las filas de la última a la primera
    for i in range(len(matrix)):  # Desde la última fila hasta la primera
        for j in range(len(matrix[i]) - 1, -1, -1):  # Desde la última columna hasta la primera
            if matrix[i][j] != -1:  # Si encontramos un valor distinto a -1
                return i, j  # Devolver la fila y la columna
    return None  # Si no hay valores diferentes a -1, retornar None


def _draw_pyramid_from_matrices(plant_matrix, side_matrix):

    anchura, profundidad = find_first_non_minus_one(plant_matrix)
    altura, _ = find_first_non_minus_one(side_matrix)
    # Usamos plt.subplots() para crear la figura y los ejes
    fig, ax = plt.subplots(figsize=(8, 4), dpi=100, subplot_kw={'projection': '3d'})

    # Definir los colores de los cubos
    colors = {0: 'red', 1: 'green', 2: 'blue', 3: 'yellow'}

    # Definir el tamaño de cada cubo
    size = 1

    # Iterar sobre las matrices para colocar los cubos
    for fila_planta in range(profundidad, -1, -1):  # Para cada fila de la planta
        for columna_planta in range(4, anchura+1):  # Para cada columna de la planta
            if plant_matrix[fila_planta][columna_planta] != -1:  # Si hay un cubo en la vista de planta
                # Obtener la altura del cubo desde la vista lateral
                height = -1
                for fila_lateral in range(altura, 5):
                    if side_matrix[fila_lateral][fila_planta] == plant_matrix[fila_planta][columna_planta]:
                        height = 4 - fila_lateral
                        break



                # Dibujar el cubo en la posición (x, y, z)
                if height != -1:
                    ax.bar3d(((len(side_matrix) * size) - 1 - columna_planta) * size, 
                            ((len(side_matrix) * size) - 1 - fila_planta) * size, 
                            height * size,  # Altura del cubo
                            size, size, size, 
                            color=colors[plant_matrix[i][j]])

    # Configurar los límites del gráfico
    ax.set_xlim([0, len(plant_matrix[0]) * size])
    ax.set_ylim([0, len(plant_matrix) * size])
    ax.set_zlim([0, len(side_matrix) * size])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Ajustar la vista para que la figura se vea al fondo
    ax.view_init(elev=30, azim=-60)

    # Mostrar la figura
    plt.show()


if __name__ in "__main__":
    top_side = [[-1, -1, -1, -1, -1],
                [-1, -1, -1, -1, -1],
                [2, 3, -1, -1, -1],
                [3, 1, -1, -1, -1],
                [-1, 2, -1, -1, -1]]

    front_side = [[-1, -1, -1, -1, -1],
                    [-1, -1, -1, -1, -1],
                    [-1, -1, -1, -1, -1],
                    [-1, 1, 2, -1, -1],
                    [2, 0, 3, -1, -1]]

    _draw_pyramid_from_matrices(top_side, front_side)
