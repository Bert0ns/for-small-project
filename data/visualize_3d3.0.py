import pandas as pd
import plotly.express as px

# Nome del tuo file
file_path = 'Building2.csv'

try:
    # Carica i dati
    df = pd.read_csv(file_path, comment='#', skipinitialspace=True)
    df.columns = df.columns.str.strip() # Pulisce i nomi delle colonne
    
    print(f"Caricati {len(df)} punti.")

    # Crea il grafico interattivo
    fig = px.scatter_3d(df, x='x', y='y', z='z',
                        color='z',             # Colora in base all'altezza
                        opacity=0.7,           # Un po' di trasparenza
                        title="Visualizzazione Droni 3D")

    # Rende i punti più piccoli per vedere meglio
    fig.update_traces(marker=dict(size=3))

    # Salva in un file HTML che puoi aprire col browser
    output_html = "mappa_droni.html"
    fig.write_html(output_html)
    
    print(f"Fatto! Apri il file '{output_html}' con il tuo browser per esplorare il grafico.")

except Exception as e:
    print(f"Errore: {e}")
