#!/usr/bin/env python3

import os
import matplotlib.pyplot as plt
import numpy as np

def carregar_dados_do_arquivo(file_path):
    dados = np.loadtxt(file_path, delimiter=',')
    # Separar x e y
    x = dados[:, 0]
    y = dados[:, 1]
    return x, y

def compare_path(teleop_data):

    # Carregar dados dos arquivos
    x1, y1 = carregar_dados_do_arquivo(teleop_data)

    # Plotar gráfico
    plt.figure(figsize=(10, 10))
    plt.plot(x1, y1, label='Caminho ensinado')

    plt.title('Caminho')
    plt.xlabel('Eixo X')
    plt.ylabel('Eixo Y')
    plt.legend()
    plt.savefig('/home/fbotathome/fbot_ws/src/shark-mb-ros/data')
    plt.grid(True)
    plt.show()

path = '/home/fbotathome/fbot_ws/src/shark-mb-ros/data/teleop_data.txt'
carregar_dados_do_arquivo(path)
compare_path(path)