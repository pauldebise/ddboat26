"""
Client UDP ddboats.


Utilisation depuis le code :


   from ddboats_client import send_gpgll, send_position, get_boats


   SERVER_IP = .. # A préciser


   # Option 1 : la trame NMEA brute du GPS
   send_gpgll("SERVER_IP", 6800, 3, "$GPGLL,4811.9417,N,00300.9333,W,123519,A,A*hh", cap=90)


   # Option 2 : la position en degrés décimaux
   send_position("SERVER_IP", 6800, 3, lat=48.199028, lon=-3.015556, cap=90)


   # Positions de TOUS les bateaux, telles que renvoyées par le serveur
   for id_, b in get_boats().items():
       print(id_, b["lat"], b["lon"], b["cap"], b["age"])


cap : degrés, 0 = Nord, sens horaire (90 = Est).
age : secondes écoulées depuis la dernière mise à jour de ce bateau.


Le serveur renvoie l'état à l'adresse d'où viennent les paquets : il faut donc
continuer à envoyer la position régulièrement (au moins une fois toutes les
30 s) pour continuer à recevoir les autres bateaux.
"""


import socket
import threading
import time


_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_lock = threading.Lock()
_boats = {}                  # id -> {"lat", "lon", "cap", "t"}  (t = instant de la position)
_listener_started = False




def _listen():
   """Thread de fond : reçoit les paquets 'ALL|id,lat,lon,cap,age|...' du serveur."""
   while True:
       try:
           data, _ = _sock.recvfrom(4096)
       except OSError:
           time.sleep(0.1)      # ex. ConnectionResetError sous Windows
           continue
       parts = data.decode("utf-8", errors="replace").strip().split("|")
       if parts[0] != "ALL":
           continue
       now = time.time()
       for p in parts[1:]:
           try:
               id_, lat, lon, cap, age = p.split(",")
               entry = {"lat": float(lat), "lon": float(lon),
                        "cap": float(cap), "t": now - float(age)}
               with _lock:
                   _boats[int(id_)] = entry
           except ValueError:
               pass




def _send(host, port, msg):
   global _listener_started
   _sock.sendto(msg.encode("utf-8"), (host, port))
   if not _listener_started:        # l'écoute démarre après le premier envoi
       _listener_started = True
       threading.Thread(target=_listen, daemon=True).start()
   return msg




def send_gpgll(host, port, boat_id, gpgll, cap):
   """Envoie id|trame GPGLL|cap (le serveur décode la trame)."""
   return _send(host, port, "{}|{}|{:.1f}".format(int(boat_id), gpgll.strip(), cap % 360))




def send_position(host, port, boat_id, lat, lon, cap):
   """Envoie id|lat|lon|cap (degrés décimaux)."""
   return _send(host, port, "{}|{:.8f}|{:.8f}|{:.1f}".format(
       int(boat_id), lat, lon, cap % 360))




def get_boats():
   """Dernier état connu de tous les bateaux : {id: {lat, lon, cap, age}}.


   Vide tant que le serveur n'a rien renvoyé (compter ~1 s après le premier envoi).
   """
   now = time.time()
   with _lock:
       return {i: {"lat": b["lat"], "lon": b["lon"], "cap": b["cap"], "age": now - b["t"]}
               for i, b in _boats.items()}
