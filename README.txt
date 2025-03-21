# Planificación de Rutas para Robots con A*

## Descripción
Este proyecto implementa un sistema de planificación de rutas para un robot en un entorno 2D con obstáculos generados aleatoriamente. Utiliza el algoritmo A* para encontrar la ruta más corta desde un punto de inicio hasta un destino evitando obstáculos.

## Requisitos
Para ejecutar el proyecto, asegúrate de tener instalado **Python 3.x** y las siguientes librerías:

```bash
pip install matplotlib networkx
```

## Cómo ejecutar el proyecto
1. Clona este repositorio o descarga los archivos.
2. Abre una terminal en la carpeta del proyecto.
3. Ejecuta el script principal:

```bash
python main.py
```

## Funcionamiento
- Se genera una cuadrícula de 10x10 representando el entorno.
- Se añaden obstáculos en posiciones aleatorias.
- El robot parte de la posición (0,0) y debe llegar a (9,9).
- Se usa el algoritmo A* con la heurística de Manhattan para encontrar la mejor ruta.
- Se muestra una visualización con el camino óptimo, obstáculos y nodos del grafo.

## Archivos principales
- **main.py**: Código principal del proyecto.
- **README.md**: Este archivo con la descripción y guía de uso.

## Mejoras futuras
- Permitir movimientos diagonales con heurística euclidiana.
- Optimizar el tiempo de ejecución en entornos más grandes.
- Implementar una interfaz gráfica para mayor interacción.

## Autor
Desarrollado por Juan Esteban Franco Estacio.

