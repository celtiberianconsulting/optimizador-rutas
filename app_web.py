import streamlit as st
import pandas as pd
import requests
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import time
import folium
import polyline
import math
from folium import Icon
from streamlit_folium import folium_static
import io

# --- CONFIGURACIÓN DE LA PÁGINA WEB ---
st.set_page_config(page_title="Optimizador Logístico", page_icon="🚚", layout="wide")

st.title("🚚 Generador de Rutas: Secuencia Maestra")
st.markdown("Sube tu archivo Excel con los clientes diarios para calcular la ruta más óptima por clústeres de entrega a pie.")

# --- FUNCIONES MATEMÁTICAS Y DE OSRM (Idénticas a tu script) ---
def haversine_distance(coord1, coord2):
    lon1, lat1 = coord1
    lon2, lat2 = coord2
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1-a)))

def get_osrm_time_matrix(coords):
    n = len(coords)
    matrix = [[0] * n for _ in range(n)]
    chunk_size = 50
    for i in range(0, n, chunk_size):
        for j in range(0, n, chunk_size):
            sources = list(range(i, min(i + chunk_size, n)))
            destinations = list(range(j, min(j + chunk_size, n)))
            req_coords = [coords[idx] for idx in sources] + [coords[idx] for idx in destinations]
            coords_str = ';'.join([f"{lon},{lat}" for lon, lat in req_coords])
            src_str = ';'.join(map(str, range(len(sources))))
            dst_str = ';'.join(map(str, range(len(sources), len(sources) + len(destinations))))
            url = f"http://router.project-osrm.org/table/v1/driving/{coords_str}?sources={src_str}&destinations={dst_str}&annotations=duration"
            for _ in range(3):
                try:
                    resp = requests.get(url, timeout=15)
                    if resp.status_code == 200:
                        durations = resp.json()['durations']
                        for row_idx, src in enumerate(sources):
                            for col_idx, dst in enumerate(destinations):
                                val = durations[row_idx][col_idx]
                                matrix[src][dst] = int(val) if val is not None else 9999999
                        break
                except:
                    time.sleep(2)
            time.sleep(0.1)
    return matrix

def get_route_geometry(coords_seq):
    chunk_size = 50
    full_geometry = []
    for i in range(0, len(coords_seq) - 1, chunk_size - 1):
        chunk = coords_seq[i:min(i + chunk_size, len(coords_seq))]
        if len(chunk) < 2: break
        coords_str = ';'.join([f"{lon},{lat}" for lon, lat in chunk])
        url = f"http://router.project-osrm.org/route/v1/driving/{coords_str}?overview=full&geometries=polyline"
        for _ in range(3):
            try:
                resp = requests.get(url, timeout=15)
                if resp.status_code == 200 and resp.json()['code'] == 'Ok':
                    full_geometry.extend(polyline.decode(resp.json()['routes'][0]['geometry']))
                    break
            except:
                time.sleep(2)
        time.sleep(0.1)
    return full_geometry

# --- INTERFAZ WEB ---
archivo_subido = st.file_uploader("Arrastra aquí tu Excel (datos_rutas.xlsx)", type=["xlsx"])

if archivo_subido is not None:
    df = pd.read_excel(archivo_subido)
    
    # Filtro de Limpieza
    df['Latitud'] = pd.to_numeric(df['Latitud'].astype(str).str.replace(',', '.'), errors='coerce')
    df['Longitud'] = pd.to_numeric(df['Longitud'].astype(str).str.replace(',', '.'), errors='coerce')
    df = df.dropna(subset=['Latitud', 'Longitud']).reset_index(drop=True)
    
    st.success(f"Archivo cargado correctamente. {len(df)-1} clientes detectados.")
    
    # --- LA CAJA FUERTE (MEMORIA DE STREAMLIT) ---
    # Inicializamos la variable que recordará si ya hemos calculado la ruta
    if 'calculo_terminado' not in st.session_state:
        st.session_state['calculo_terminado'] = False

    # Al pulsar el botón, hacemos los cálculos pesados
    if st.button("🚀 Calcular Ruta Óptima", type="primary"):
        st.session_state['calculo_terminado'] = False # Reseteamos por si ha subido un Excel nuevo
        
        with st.spinner('Fase 1: Agrupando clientes a menos de 80 metros...'):
            all_coords = list(zip(df['Longitud'], df['Latitud']))
            DIST_THRESHOLD = 80 
            
            stops = {0: {'coords': all_coords[0], 'client_indices': [0]}}
            stop_counter = 1
            
            for idx in range(1, len(all_coords)):
                assigned = False
                for s_id in range(1, stop_counter):
                    ref_idx = stops[s_id]['client_indices'][0]
                    if haversine_distance(all_coords[idx], all_coords[ref_idx]) <= DIST_THRESHOLD:
                        stops[s_id]['client_indices'].append(idx)
                        assigned = True
                        break
                if not assigned:
                    stops[stop_counter] = {'coords': all_coords[idx], 'client_indices': [idx]}
                    stop_counter += 1
            
            stop_coords = [stops[i]['coords'] for i in range(stop_counter)]
            
        with st.spinner('Fase 2: Consultando estado del tráfico (OSRM)...'):
            time_matrix = get_osrm_time_matrix(stop_coords)
            
        with st.spinner('Fase 3: Ejecutando IA de enrutamiento (aprox 60s)...'):
            manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, 0)
            routing = pywrapcp.RoutingModel(manager)
            
            def time_callback(from_index, to_index):
                return time_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
                
            transit_callback_index = routing.RegisterTransitCallback(time_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
            
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
            search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
            search_parameters.time_limit.seconds = 60
            
            solution = routing.SolveWithParameters(search_parameters)
            
        if solution:
            export_data = []
            depot_lat, depot_lon = df.iloc[0]['Latitud'], df.iloc[0]['Longitud']
            m = folium.Map(location=[depot_lat, depot_lon], zoom_start=14)
            
            folium.Marker([depot_lat, depot_lon], popup="Almacén Central", icon=Icon(color='black', icon='home', prefix='fa')).add_to(m)

            index = routing.Start(0)
            route_stop_coords = []
            visit_order = 1
            
            while not routing.IsEnd(index):
                stop_index = manager.IndexToNode(index)
                current_stop_info = stops[stop_index]
                route_stop_coords.append(current_stop_info['coords'])
                
                client_indices = current_stop_info['client_indices']
                stop_lat_lon = [current_stop_info['coords'][1], current_stop_info['coords'][0]]
                
                if len(client_indices) == 1:
                    cliente_unico_idx = client_indices[0]
                    if cliente_unico_idx != 0:
                        folium.Marker(stop_lat_lon, popup=f"Orden: {visit_order} - {df.iloc[cliente_unico_idx]['ID_Cliente']}", icon=Icon(color='blue', icon='truck', prefix='fa')).add_to(m)
                        export_data.append({
                            'Orden_Maestro': visit_order, 'Tipo': 'Parada Única (AZUL)', 'ID_Cliente': df.iloc[cliente_unico_idx]['ID_Cliente'],
                            'Lat': stop_lat_lon[0], 'Lon': stop_lat_lon[1]
                        })
                        visit_order += 1
                elif len(client_indices) > 1:
                    folium.Marker(stop_lat_lon, popup=f"Orden: {visit_order} - Clúster de {len(client_indices)} clientes", icon=Icon(color='green', icon='users', prefix='fa')).add_to(m)
                    for client_idx in client_indices:
                        if client_idx == 0: continue 
                        c_data = df.iloc[client_idx]
                        folium.CircleMarker([c_data['Latitud'], c_data['Longitud']], radius=5, color='red', fill=True, popup=c_data['ID_Cliente']).add_to(m)
                        export_data.append({
                            'Orden_Maestro': visit_order, 'Tipo': 'Clúster (VERDE/ROJO)', 'ID_Cliente': c_data['ID_Cliente'],
                            'Lat': c_data['Latitud'], 'Lon': c_data['Longitud']
                        })
                        visit_order += 1
                index = solution.Value(routing.NextVar(index))
                
            route_stop_coords.append((depot_lon, depot_lat))
            geom = get_route_geometry(route_stop_coords)
            if geom: folium.PolyLine(geom, color='green', weight=4, opacity=0.75).add_to(m)
            
            # Generar Excel en memoria para descarga
            df_export = pd.DataFrame(export_data)
            output = io.BytesIO()
            with pd.ExcelWriter(output, engine='openpyxl') as writer:
                df_export.to_excel(writer, index=False, sheet_name='Ruta_Optima')
            
            # GUARDAMOS TODO EN LA CAJA FUERTE
            st.session_state['mapa'] = m
            st.session_state['excel_data'] = output.getvalue()
            st.session_state['calculo_terminado'] = True
            
        else:
            st.error("No se pudo hallar una solución. Verifica los datos del Excel.")

    # --- MOSTRAR RESULTADOS (Fuera de la lógica del botón) ---
    # Si la caja fuerte dice que ya hemos terminado, mostramos los entregables de forma permanente
    if st.session_state.get('calculo_terminado', False):
        st.success("¡Ruta calculada con éxito!")
        
        st.markdown("### Mapa Operativo")
        folium_static(st.session_state['mapa'], width=900, height=500)
        
        st.download_button(
            label="📥 Descargar Hoja de Ruta (Excel)",
            data=st.session_state['excel_data'],
            file_name="secuencia_maestra.xlsx",
            mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"
        )
