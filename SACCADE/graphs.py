import pandas as pd
import plotly.graph_objects as go

# Leer el archivo CSV
data = pd.read_csv('/home/brairlab/Desktop/iCUB/SACCADE/data/head_position.csv', header=None, names=['Label', 'X', 'Y'])

# Convertir las columnas X y Y a cadenas de texto y extraer los valores numéricos
data['X'] = data['X'].astype(str).str.extract('(\d+\.\d+)').astype(float)
data['Y'] = data['Y'].astype(str).str.extract('(\d+\.\d+)').astype(float)

# Crear la figura con Plotly
fig = go.Figure()

# Añadir la traza de la posición X (línea azul)
fig.add_trace(go.Scatter(x=data.index, y=data['X'], mode='lines+markers', name='Head_position_X', line=dict(color='blue', width=2), marker=dict(size=6, color='blue')))

# Añadir la traza de la posición Y (línea roja)
fig.add_trace(go.Scatter(x=data.index, y=data['Y'], mode='lines+markers', name='Head_position_Y', line=dict(color='red', width=2), marker=dict(size=6, color='red')))

# Añadir título y etiquetas
fig.update_layout(
    title='Head Over Time',
    xaxis_title='Iterations',  # Eje X para el número de iteraciones
    yaxis_title='Head Position',  # Eje Y para la posición de los ojos
    xaxis=dict(showticklabels=False),  # No mostrar etiquetas en el eje X
    showlegend=True,  # Mostrar la leyenda
    template='simple_white'  # Puedes cambiar el tema si lo deseas
)

# Mostrar la gráfica
fig.show()
