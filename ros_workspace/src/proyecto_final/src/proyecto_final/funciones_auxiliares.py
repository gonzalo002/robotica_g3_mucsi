def crear_mensaje(mensaje:str, tipo:str='ERROR', nombre_clase:str="", tkinter:bool=True):
    # --- CÓDIGO COLORES ---
    c = {
        "ERROR":   "\033[31m",  # Rojo
        "SUCCESS": "\033[32m",  # Verde
        "WARN":    "\033[33m",  # Amarillo
        "INFO":    "\033[0m" ,  # Restablecer
        "RESET":   "\033[0m" ,  # Restablecer
        "INPUT":   "\033[35m",  # Magenta
    }

    msg_sin_tipo = f"[{tipo.rjust(7)}] [{nombre_clase.rjust(20)}]: {mensaje}"
    msg = f"{c[tipo]}{msg_sin_tipo}{c['RESET']}"
    if tipo == "INPUT" and not tkinter:
        return input(msg)
    else:
        print(msg)
    
    return msg_sin_tipo
