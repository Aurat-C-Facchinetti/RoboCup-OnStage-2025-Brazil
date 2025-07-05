import socket



def connessione_bluetooth(HC05_MAC,PORT):
    sock=None
    try:
        print(f"Connessione a {HC05_MAC} sulla porta {PORT}...")

        # Crea un socket Bluetooth RFCOMM
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        # Connessione al modulo Bluetooth
        sock.connect((HC05_MAC, PORT))
        print("Connessione riuscita!")

        return sock
    except Exception as e:
        print(f"Errore nella connessione Bluetooth: {e}")
        return None

def flush_buffer(sock):
    try:
        sock.settimeout(0.1)  # Imposta un timeout breve
        while True:
            if not sock.recv(1024):
                break
    except:
        pass
    finally:
        sock.settimeout(None)  # Ripristina il timeout

def invia_messaggio(msg,sock):
    
    try:
        sock.send(msg.encode())
        print(f"Messaggio inviato: {msg}")
    except Exception as e:
        print(f"Errore nell'invio del messaggio: {e}")

def ricevi_messaggio(sock):
    try:
        data = b""  # Accumula i dati ricevuti
        while True:
            chunk = sock.recv(1)  # Riceve un byte alla volta
            if not chunk:
                break
            data += chunk
            if chunk == b"\n":  # Interrompe se trova il carattere di nuova riga
                break
        return data.decode().strip()
    except Exception as e:
        print(f"Errore nella ricezione del messaggio: {e}")
        return None


def chiudi_connessione(sock):
    
    if sock:
        sock.close()
        print("Connessione chiusa.")



