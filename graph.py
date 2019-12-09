import pandas as pd
import plotly.graph_objects as go

df = pd.read_csv('/home/alexander/catkin/src/manatee_PID/depth.csv')

fig = go.Figure(go.Scatter(x = df['TIME'], y = df['DEPTH'],
                  name=''))

fig.update_layout(title='PID Controller Graph',
                   plot_bgcolor='rgb(230, 230,230)',
                   showlegend=True)

fig.show()
