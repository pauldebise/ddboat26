import math

def convertir_nmea_en_decimal(coord_nmea):
  """Convertit une coordonnée NMEA (ex: 4812.345) en degrés décimaux."""
  degres = int(coord_nmea // 100)
  minutes = coord_nmea % 100
  return degres + (minutes / 60.0)

def calculer_distance_gps(lat1, lon1, lat2, lon2):
    # Rayon moyen de la Terre en mètres
    R = 6371000.0

    # Conversion des degrés en radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Différences de coordonnées
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Formule de Haversine
    a = ( math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance en mètres
    distance = R * c
    return distance




if __name__ == "__main__":
    from ddboatclass import *
    from environnementclass import Environnement
    from imu9_driver_v2 import Imu9IO
    from gps_driver_v2 import GpsIO
    from arduino_driver_v2 import ArduinoIO

    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    ddb = DDBoat(ard, gps, imu)

    lac = Environnement()

    while True:
        rayon_zone = 5
        pos_ddb = ddb.get_DDboat_cart_position(lac.home_latlong)
        pos_keep_point = ddb.convert_cart_position(lac.station_keeping_point[0],lac.station_keeping_point[1], lac.home_latlong)


        if dist_cart(pos_ddb, pos_keep_point) < rayon_zone:
            continue

        else:
            ddb.rejoindre_keeping_zone(pos_keep_point, lac.home_latlong)
        time.sleep(0.01)