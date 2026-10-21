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
    home = lac.home_latlong

    # pour jalon 3, définition du point autour duquel on doit rester statique
    coord_station_keeping_point = np.array([48.199111, -3.015750])
    lac.assignation_keeping_point(coord_station_keeping_point)

    rayon_zone = 5
    pos_keep_point = ddb.convert_cart_position(lac.station_keeping_point[0], lac.station_keeping_point[1],
                                               home)

    while True:
        pos_ddb = ddb.get_DDboat_cart_position(home)

        if dist_cart(pos_ddb, pos_keep_point) < rayon_zone:
            continue

        else:
            ddb.rejoindre_keeping_zone(pos_keep_point, home)
        time.sleep(0.01)