#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IHM minimaliste : lac, bateau, points de bordure et champ de gradient.

L'interface s'ouvre dans un navigateur (locale ou distante) et affiche :

- le plan d'eau du Lac de Guerledan dans le repere local de la base ;
- les six points du couloir et des bordures ;
- la position du bateau (simulation ou GPS) ;
- le champ de gradient calcule par Environnement.direction().

Usages
------
    python3 ihm_lac.py
        Simulation : un bateau parcourt le gradient depuis le couloir
        jusqu'a la base, puis recommence.

    python3 ihm_lac.py --live
        Lecture du GPS reel (port /dev/ttyGPS0 par defaut).

    python3 ihm_lac.py --live --gps 1 --host 0.0.0.0 --port 8080
        GPS sur /dev/ttyGPS1, interface accessible depuis le reseau local.

    python3 ihm_lac.py --mission --host 0.0.0.0 --no-browser
        Affiche la telemetrie envoyee par mission_sortie_retour.py (UDP 5006)
        sans ouvrir le GPS.

Commandes
---------
Molette ou pincement : zoom ; glisser : deplacer.
Touches 0 / L : revoir les points / le lac complet ; F : champ ; T : trace.

Le script ne modifie aucun fichier existant. Le fond de lac est derive
d'OpenStreetMap (way 348334456, Lac de Guerledan), (c) OpenStreetMap
contributors, licence ODbL.
"""

import argparse
import contextlib
import io
import json
import math
import socket
import sys
import threading
import time
import types
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from socketserver import ThreadingMixIn
from urllib.parse import urlparse

BASE_DIR = Path(__file__).resolve().parent
DRIVERS_DIR = BASE_DIR / "drivers"
R_TERRE = 6371000.0

try:
    from http.server import ThreadingHTTPServer
except ImportError:  # Python 3.5
    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True


# Rivage du lac en latitude/longitude (polygone simplifie a 2 m).
# Il est converti dans le repere local de la base au demarrage.
SHORE_TEXT = """48.2072267,-3.1096450 48.2072174,-3.1097313 48.2072549,-3.1097411 48.2072797,-3.1098402 48.2075047,-3.1097886 48.2080295,-3.1095493 48.2084763,-3.1091115 48.2086842,-3.1087669 48.2088366,-3.1079904 48.2087161,-3.1073550 48.2085000,-3.1070000 48.2080148,-3.1065720 48.2074664,-3.1064127 48.2070187,-3.1064449 48.2068533,-3.1063834 48.2063063,-3.1064380 48.2059296,-3.1062509 48.2058194,-3.1059963 48.2055858,-3.1058229 48.2056471,-3.1050219 48.2060531,-3.1030369 48.2062751,-3.1024608 48.2064869,-3.1016583 48.2066975,-3.1006766 48.2067774,-3.0998593 48.2068277,-3.0987573 48.2067228,-3.0977761 48.2067398,-3.0967608 48.2063161,-3.0958809 48.2052510,-3.0944223 48.2041442,-3.0937504 48.2037893,-3.0937857 48.2034034,-3.0938958 48.2032619,-3.0943352 48.2028991,-3.0939106 48.2024863,-3.0936652 48.2024354,-3.0930385 48.2025477,-3.0927275 48.2030296,-3.0909767 48.2032947,-3.0901279 48.2036849,-3.0895649 48.2039774,-3.0889408 48.2041210,-3.0882208 48.2043928,-3.0872799 48.2045841,-3.0868342 48.2067396,-3.0867800 48.2069022,-3.0868149 48.2070685,-3.0869302 48.2072329,-3.0869490 48.2074131,-3.0868455 48.2077893,-3.0868855 48.2080098,-3.0867177 48.2081312,-3.0864091 48.2089928,-3.0857361 48.2091413,-3.0854976 48.2092776,-3.0855065 48.2092931,-3.0858351 48.2094507,-3.0860906 48.2094713,-3.0862210 48.2095529,-3.0863449 48.2097378,-3.0864935 48.2101344,-3.0869238 48.2104818,-3.0876056 48.2110294,-3.0878812 48.2110603,-3.0876604 48.2107984,-3.0867208 48.2108709,-3.0866226 48.2109050,-3.0864525 48.2109832,-3.0854097 48.2109779,-3.0846351 48.2109322,-3.0843685 48.2111052,-3.0837690 48.2112381,-3.0831174 48.2114462,-3.0827553 48.2118545,-3.0827350 48.2122165,-3.0819690 48.2124241,-3.0810656 48.2125097,-3.0805096 48.2126306,-3.0801471 48.2131038,-3.0795562 48.2133237,-3.0793182 48.2137151,-3.0794006 48.2139117,-3.0796589 48.2140887,-3.0797985 48.2141048,-3.0794060 48.2140416,-3.0791128 48.2137754,-3.0787460 48.2135275,-3.0785936 48.2129916,-3.0785746 48.2127232,-3.0784703 48.2124116,-3.0786491 48.2120949,-3.0784843 48.2117174,-3.0778463 48.2115926,-3.0774837 48.2111376,-3.0772090 48.2109961,-3.0770620 48.2108746,-3.0765859 48.2116122,-3.0755616 48.2118966,-3.0750733 48.2121203,-3.0742744 48.2121858,-3.0734205 48.2110484,-3.0750123 48.2104888,-3.0755363 48.2099039,-3.0759104 48.2096514,-3.0759089 48.2094015,-3.0756024 48.2096304,-3.0749428 48.2099562,-3.0742172 48.2100136,-3.0739725 48.2098349,-3.0740043 48.2091450,-3.0743830 48.2089805,-3.0747392 48.2087605,-3.0746266 48.2085760,-3.0742043 48.2077614,-3.0729987 48.2065476,-3.0704159 48.2064719,-3.0685072 48.2069354,-3.0664001 48.2069328,-3.0645034 48.2070267,-3.0642920 48.2069496,-3.0625319 48.2070209,-3.0624095 48.2069410,-3.0618802 48.2070483,-3.0613518 48.2069461,-3.0610380 48.2072092,-3.0606061 48.2068894,-3.0604586 48.2068200,-3.0601877 48.2068903,-3.0600250 48.2069793,-3.0599767 48.2069103,-3.0598015 48.2068020,-3.0597210 48.2068995,-3.0591041 48.2068335,-3.0590156 48.2068024,-3.0588982 48.2072894,-3.0577415 48.2076899,-3.0555367 48.2079678,-3.0550924 48.2082795,-3.0549610 48.2082895,-3.0547679 48.2079853,-3.0544075 48.2075488,-3.0543378 48.2074251,-3.0539194 48.2070714,-3.0532596 48.2069010,-3.0522761 48.2069018,-3.0518648 48.2067445,-3.0517817 48.2065486,-3.0513435 48.2065435,-3.0511657 48.2061445,-3.0506208 48.2060059,-3.0501439 48.2058665,-3.0498527 48.2057221,-3.0496743 48.2056490,-3.0495210 48.2058643,-3.0488703 48.2058492,-3.0487827 48.2056633,-3.0483969 48.2053486,-3.0479854 48.2052744,-3.0472112 48.2052941,-3.0469296 48.2054477,-3.0465179 48.2055938,-3.0462870 48.2058331,-3.0461918 48.2057667,-3.0460383 48.2058441,-3.0459781 48.2059222,-3.0461503 48.2062897,-3.0458731 48.2063009,-3.0457857 48.2066302,-3.0457070 48.2072230,-3.0465940 48.2073613,-3.0467000 48.2074142,-3.0468921 48.2075215,-3.0470552 48.2082320,-3.0478298 48.2083646,-3.0480282 48.2085082,-3.0485845 48.2085951,-3.0487159 48.2096054,-3.0494833 48.2100456,-3.0495587 48.2101311,-3.0495250 48.2101894,-3.0494485 48.2107201,-3.0494388 48.2107790,-3.0493194 48.2106883,-3.0491621 48.2105813,-3.0490944 48.2102043,-3.0486513 48.2100357,-3.0485501 48.2096800,-3.0481507 48.2093871,-3.0476826 48.2091432,-3.0470423 48.2089408,-3.0468273 48.2087646,-3.0467128 48.2084408,-3.0466822 48.2085002,-3.0460983 48.2085689,-3.0458542 48.2086123,-3.0457992 48.2088342,-3.0457513 48.2093873,-3.0450813 48.2095829,-3.0447876 48.2096251,-3.0446231 48.2097533,-3.0444496 48.2099448,-3.0440655 48.2103841,-3.0425939 48.2105049,-3.0417255 48.2105019,-3.0415380 48.2105632,-3.0413463 48.2106165,-3.0412735 48.2106429,-3.0411229 48.2104423,-3.0408916 48.2103916,-3.0403579 48.2102778,-3.0405555 48.2102260,-3.0408158 48.2102747,-3.0402767 48.2101873,-3.0405729 48.2101316,-3.0409199 48.2093321,-3.0433407 48.2089106,-3.0440006 48.2079813,-3.0449581 48.2077971,-3.0450198 48.2076754,-3.0447972 48.2074917,-3.0443037 48.2068903,-3.0438272 48.2065976,-3.0436921 48.2066265,-3.0428955 48.2064904,-3.0428687 48.2063115,-3.0431637 48.2054648,-3.0422142 48.2053940,-3.0412496 48.2050674,-3.0401194 48.2049460,-3.0394462 48.2050078,-3.0390904 48.2049532,-3.0381587 48.2046791,-3.0372233 48.2046896,-3.0369017 48.2045111,-3.0364427 48.2044986,-3.0362684 48.2042500,-3.0357500 48.2032500,-3.0340000 48.2009322,-3.0323685 48.2007474,-3.0320024 48.2005179,-3.0318605 48.1987500,-3.0300000 48.1983695,-3.0290131 48.1985454,-3.0279048 48.1984799,-3.0275299 48.1980476,-3.0269333 48.1978281,-3.0259399 48.1977934,-3.0249914 48.1979043,-3.0236771 48.1985647,-3.0228108 48.1988153,-3.0214767 48.1987399,-3.0195865 48.1986546,-3.0186606 48.1989124,-3.0169064 48.1989322,-3.0165426 48.1990379,-3.0165473 48.1991308,-3.0166546 48.1991918,-3.0168039 48.1995290,-3.0168829 48.2001215,-3.0172091 48.2002224,-3.0173441 48.2003703,-3.0174375 48.2004647,-3.0173197 48.2007811,-3.0174054 48.2008943,-3.0175852 48.2012609,-3.0177114 48.2014717,-3.0177004 48.2016162,-3.0176182 48.2018050,-3.0173434 48.2019222,-3.0176944 48.2018979,-3.0178163 48.2020671,-3.0183891 48.2018216,-3.0188563 48.2017363,-3.0192093 48.2019027,-3.0192655 48.2023050,-3.0189045 48.2024493,-3.0189316 48.2026913,-3.0192939 48.2029497,-3.0197852 48.2031214,-3.0198065 48.2030671,-3.0190653 48.2029735,-3.0183851 48.2030414,-3.0169311 48.2037917,-3.0153371 48.2038457,-3.0153554 48.2039225,-3.0152393 48.2040415,-3.0154483 48.2042023,-3.0155036 48.2047765,-3.0153199 48.2052139,-3.0149203 48.2057625,-3.0151760 48.2062510,-3.0151201 48.2063686,-3.0155168 48.2065106,-3.0164599 48.2067321,-3.0183594 48.2072303,-3.0200409 48.2089362,-3.0230320 48.2089303,-3.0234277 48.2092577,-3.0245510 48.2099049,-3.0249638 48.2110000,-3.0245000 48.2116823,-3.0244601 48.2121895,-3.0230423 48.2128164,-3.0218752 48.2130310,-3.0210360 48.2129350,-3.0203965 48.2126196,-3.0196157 48.2125236,-3.0192357 48.2122203,-3.0189972 48.2115457,-3.0187730 48.2113382,-3.0185673 48.2112842,-3.0180241 48.2116489,-3.0166626 48.2119270,-3.0160168 48.2121670,-3.0152454 48.2122278,-3.0142213 48.2118980,-3.0134496 48.2117268,-3.0126873 48.2117373,-3.0121222 48.2114413,-3.0118404 48.2112178,-3.0117029 48.2105761,-3.0111501 48.2102090,-3.0107434 48.2099865,-3.0103487 48.2098489,-3.0113167 48.2097718,-3.0113721 48.2097631,-3.0131544 48.2099087,-3.0140214 48.2097050,-3.0151778 48.2095306,-3.0169134 48.2093075,-3.0171112 48.2090144,-3.0172136 48.2086296,-3.0174712 48.2084792,-3.0177212 48.2081950,-3.0178497 48.2078870,-3.0178528 48.2078280,-3.0179134 48.2077110,-3.0176792 48.2075833,-3.0167191 48.2075352,-3.0159890 48.2075695,-3.0156936 48.2074912,-3.0151613 48.2073718,-3.0150416 48.2073115,-3.0146930 48.2073117,-3.0142471 48.2071142,-3.0142201 48.2068680,-3.0139480 48.2066322,-3.0133828 48.2063641,-3.0132100 48.2062137,-3.0132552 48.2059827,-3.0130931 48.2058513,-3.0128503 48.2055150,-3.0128749 48.2053019,-3.0131458 48.2052059,-3.0130293 48.2050389,-3.0129922 48.2049216,-3.0133815 48.2048781,-3.0134357 48.2047178,-3.0133152 48.2046515,-3.0132023 48.2045305,-3.0131399 48.2040623,-3.0133517 48.2039420,-3.0132442 48.2038707,-3.0129857 48.2038274,-3.0129894 48.2036278,-3.0132489 48.2032935,-3.0138764 48.2030565,-3.0141141 48.2029279,-3.0141669 48.2024205,-3.0140662 48.2017179,-3.0140289 48.2015281,-3.0142152 48.2014988,-3.0143576 48.2015020,-3.0146008 48.2013888,-3.0150684 48.2013726,-3.0152471 48.2011715,-3.0152529 48.2007882,-3.0150138 48.2005788,-3.0150128 48.2003302,-3.0148873 48.2000014,-3.0148398 48.1995598,-3.0144665 48.1993020,-3.0143922 48.1992154,-3.0143846 48.1991433,-3.0144392 48.1990520,-3.0146821 48.1990097,-3.0144872 48.1987750,-3.0144891 48.1984583,-3.0146990 48.1984250,-3.0144740 48.1984463,-3.0143950 48.1983316,-3.0137143 48.1981095,-3.0129955 48.1980387,-3.0126575 48.1978776,-3.0123529 48.1978013,-3.0123941 48.1977531,-3.0121862 48.1975831,-3.0122190 48.1974899,-3.0123074 48.1974575,-3.0125247 48.1974665,-3.0127734 48.1974234,-3.0129132 48.1973662,-3.0129449 48.1973233,-3.0129271 48.1973481,-3.0127952 48.1972736,-3.0128141 48.1968841,-3.0126745 48.1966910,-3.0126686 48.1965709,-3.0128245 48.1962732,-3.0133655 48.1962012,-3.0134162 48.1959386,-3.0137949 48.1957996,-3.0139291 48.1957382,-3.0140643 48.1956616,-3.0141095 48.1956197,-3.0142067 48.1954770,-3.0143575 48.1954322,-3.0145866 48.1950079,-3.0152652 48.1948298,-3.0154063 48.1946557,-3.0156572 48.1944247,-3.0162488 48.1941854,-3.0164854 48.1941033,-3.0166826 48.1940473,-3.0167334 48.1939793,-3.0166980 48.1939455,-3.0167368 48.1941175,-3.0169934 48.1941542,-3.0169281 48.1942894,-3.0171238 48.1942446,-3.0172005 48.1944761,-3.0175104 48.1954097,-3.0185170 48.1953729,-3.0185992 48.1954158,-3.0186821 48.1954764,-3.0186609 48.1954437,-3.0187358 48.1954789,-3.0188037 48.1955548,-3.0187770 48.1959304,-3.0183237 48.1960385,-3.0183842 48.1962077,-3.0188094 48.1963032,-3.0192454 48.1963172,-3.0196723 48.1961952,-3.0206559 48.1955164,-3.0225388 48.1954421,-3.0228610 48.1953583,-3.0242266 48.1952727,-3.0243799 48.1952102,-3.0248496 48.1951602,-3.0256051 48.1951638,-3.0261503 48.1952077,-3.0264307 48.1952053,-3.0269385 48.1950815,-3.0276241 48.1950425,-3.0281716 48.1951019,-3.0290706 48.1952027,-3.0293257 48.1952575,-3.0297206 48.1953558,-3.0299977 48.1954038,-3.0302531 48.1954864,-3.0304357 48.1956481,-3.0306320 48.1957385,-3.0309181 48.1959009,-3.0311777 48.1961547,-3.0314123 48.1964775,-3.0318913 48.1967121,-3.0323211 48.1970358,-3.0326254 48.1973757,-3.0328302 48.1978277,-3.0333743 48.1983319,-3.0337604 48.1985389,-3.0341877 48.1986660,-3.0345670 48.1989128,-3.0347693 48.1990420,-3.0350667 48.1991261,-3.0351608 48.1992961,-3.0355985 48.1999257,-3.0354036 48.2001687,-3.0355184 48.2004020,-3.0358497 48.2007985,-3.0362488 48.2013141,-3.0369000 48.2014283,-3.0371203 48.2016742,-3.0374516 48.2017653,-3.0376734 48.2019901,-3.0383901 48.2020217,-3.0395484 48.2019815,-3.0398484 48.2020443,-3.0399701 48.2020882,-3.0401809 48.2021007,-3.0405046 48.2021409,-3.0406816 48.2022362,-3.0416152 48.2021772,-3.0422533 48.2019721,-3.0430584 48.2018868,-3.0441113 48.2018688,-3.0448064 48.2016999,-3.0468619 48.2018122,-3.0485601 48.2018622,-3.0488699 48.2023085,-3.0497877 48.2025360,-3.0501269 48.2026164,-3.0504787 48.2027436,-3.0507715 48.2029438,-3.0508742 48.2031031,-3.0510135 48.2032714,-3.0514183 48.2037918,-3.0523800 48.2038696,-3.0525776 48.2039000,-3.0527736 48.2040477,-3.0529409 48.2041160,-3.0529274 48.2046440,-3.0532879 48.2051191,-3.0538971 48.2054452,-3.0545352 48.2057277,-3.0556716 48.2056246,-3.0558227 48.2055531,-3.0558566 48.2054216,-3.0561060 48.2053599,-3.0567751 48.2052270,-3.0572871 48.2050496,-3.0576906 48.2050288,-3.0585972 48.2049646,-3.0590165 48.2049510,-3.0593539 48.2047787,-3.0596878 48.2047327,-3.0600409 48.2045847,-3.0605418 48.2044148,-3.0622836 48.2043939,-3.0637664 48.2043448,-3.0643626 48.2044050,-3.0653082 48.2043860,-3.0656642 48.2044973,-3.0661690 48.2043513,-3.0666172 48.2038500,-3.0674927 48.2035686,-3.0675506 48.2027727,-3.0680162 48.2023788,-3.0685137 48.2022478,-3.0688688 48.2023439,-3.0690255 48.2024584,-3.0691004 48.2033042,-3.0689673 48.2036903,-3.0693089 48.2038535,-3.0693499 48.2041357,-3.0693240 48.2042862,-3.0695130 48.2042869,-3.0699718 48.2044339,-3.0710220 48.2046391,-3.0714815 48.2047346,-3.0720493 48.2049174,-3.0726262 48.2051000,-3.0736649 48.2056279,-3.0744762 48.2059659,-3.0752673 48.2064917,-3.0760670 48.2075292,-3.0772543 48.2081052,-3.0778151 48.2090314,-3.0785887 48.2093973,-3.0786236 48.2097244,-3.0787091 48.2097247,-3.0788343 48.2093507,-3.0791466 48.2092002,-3.0791996 48.2090233,-3.0797335 48.2087504,-3.0816059 48.2087332,-3.0821600 48.2088098,-3.0829618 48.2084597,-3.0834046 48.2082243,-3.0838907 48.2074010,-3.0845212 48.2069650,-3.0847523 48.2062725,-3.0848353 48.2062226,-3.0847634 48.2062109,-3.0846350 48.2062439,-3.0845644 48.2060795,-3.0842778 48.2059060,-3.0841860 48.2055605,-3.0840427 48.2052515,-3.0839860 48.2050348,-3.0839844 48.2045833,-3.0840817 48.2043374,-3.0841905 48.2036921,-3.0846911 48.2032742,-3.0852510 48.2029965,-3.0858677 48.2025891,-3.0870338 48.2021170,-3.0879621 48.2019684,-3.0884933 48.2015545,-3.0905895 48.2015127,-3.0912043 48.2015173,-3.0917893 48.2013476,-3.0927726 48.2012826,-3.0934328 48.2012812,-3.0941132 48.2013295,-3.0945346 48.2013890,-3.0946885 48.2015902,-3.0950556 48.2017493,-3.0952693 48.2021845,-3.0956769 48.2027093,-3.0958620 48.2028079,-3.0957319 48.2037652,-3.0955486 48.2049066,-3.0961070 48.2056267,-3.0973263 48.2057434,-3.0976093 48.2058164,-3.0978596 48.2059322,-3.0988863 48.2058630,-3.0999270 48.2051870,-3.1032428 48.2050977,-3.1040579 48.2050044,-3.1045325 48.2049496,-3.1052468 48.2049745,-3.1058499 48.2052036,-3.1065316 48.2055012,-3.1069151 48.2059801,-3.1073028 48.2061462,-3.1073865 48.2064641,-3.1074455 48.2073303,-3.1072578 48.2078148,-3.1072581 48.2079958,-3.1073742 48.2081070,-3.1075270 48.2081480,-3.1077262 48.2081476,-3.1080071 48.2081171,-3.1081382 48.2080069,-3.1083922 48.2074302,-3.1093707 48.2072267,-3.1096450"""


PAGE_TEMPLATE = r"""<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Retour à la base - Lac de Guerlédan</title>
<style>
  :root {
    --land: #f4f1e8;
    --water: #cfe4e7;
    --ink: #1f302f;
    --muted: #70807f;
    --line: rgba(31, 48, 47, 0.11);
    --card: rgba(255, 255, 255, 0.74);
    --amber: #bf7a21;
    --green: #2f7d5a;
    --blue: #35669b;
    --base: #a53d26;
    --boat: #d4552f;
    --field: #4d8093;
    --sans: "Avenir Next", "Segoe UI", system-ui, -apple-system, sans-serif;
    --mono: "SFMono-Regular", Consolas, "Liberation Mono", Menlo, monospace;
  }
  * { box-sizing: border-box; }
  html, body { margin: 0; padding: 0; height: 100%; overflow: hidden; }
  body {
    background: var(--land);
    color: var(--ink);
    font-family: var(--sans);
    display: flex;
    flex-direction: column;
    height: 100vh;
    height: 100dvh;
    -webkit-font-smoothing: antialiased;
  }
  header {
    flex: 0 0 auto;
    height: 58px;
    padding: 0 20px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
    border-bottom: 1px solid var(--line);
    background: rgba(244, 241, 232, 0.95);
    z-index: 20;
  }
  .brand { display: flex; align-items: center; gap: 11px; min-width: 0; }
  .brand-mark {
    width: 26px; height: 26px; border: 1px solid rgba(31,48,47,.3);
    border-radius: 50%; display: flex; align-items: center; justify-content: center;
    font-size: 13px; color: var(--base); flex: 0 0 auto;
  }
  .brand-title { font-size: 15px; font-weight: 600; letter-spacing: -.01em; line-height: 1.15; }
  .brand-sub { font-size: 10px; color: var(--muted); letter-spacing: .055em; text-transform: uppercase; margin-top: 2px; }
  .topbar-right { display: flex; align-items: center; gap: 14px; flex: 0 0 auto; }
  .pill {
    display: inline-flex; align-items: center; gap: 7px;
    height: 25px; padding: 0 10px; border-radius: 99px;
    font-size: 10px; font-weight: 600; letter-spacing: .08em;
    border: 1px solid rgba(31,48,47,.12); background: rgba(255,255,255,.55);
    white-space: nowrap;
  }
  .pill-dot { width: 6px; height: 6px; border-radius: 50%; background: var(--amber); }
  .pill.live .pill-dot { background: var(--green); }
  .pill.stale .pill-dot { background: #c2412d; }
  .pill.stale { color: #a13a28; border-color: rgba(193,65,45,.3); }
  .clock { font-family: var(--mono); font-size: 11px; color: var(--muted); font-variant-numeric: tabular-nums; }
  main {
    flex: 1 1 auto;
    position: relative;
    min-height: 0;
    overflow: hidden;
    background: var(--land);
  }
  #map {
    display: block; width: 100%; height: 100%;
    touch-action: none; cursor: grab; outline: none;
  }
  #map.dragging { cursor: grabbing; }
  .card {
    position: absolute; z-index: 5;
    background: var(--card);
    border: 1px solid var(--line);
    border-radius: 12px;
    box-shadow: 0 5px 18px rgba(31,48,47,.055);
    -webkit-backdrop-filter: blur(9px);
    backdrop-filter: blur(9px);
    pointer-events: none;
  }
  .inset {
    left: 18px; top: 16px; width: 208px; padding: 9px 10px 7px;
    pointer-events: auto; cursor: pointer;
  }
  .inset:hover { background: rgba(255, 255, 255, 0.92); }
  .inset-head {
    display: flex; align-items: baseline; justify-content: space-between;
    font-size: 9px; font-weight: 600; letter-spacing: .095em; text-transform: uppercase;
    color: var(--ink); margin-bottom: 6px;
  }
  .inset-head span { font-weight: 400; letter-spacing: .03em; text-transform: none; color: var(--muted); }
  #inset { display: block; width: 100%; height: 103px; }
  .inset-credit { display: none; margin-top: 3px; font-size: 7px; line-height: 1; color: var(--muted); text-align: right; letter-spacing: .01em; }
  .telemetry {
    left: 18px; bottom: 18px; width: 230px; padding: 13px 15px 11px;
  }
  .hero { display: flex; align-items: baseline; gap: 5px; }
  .hero-value {
    font-size: 34px; line-height: .96; font-weight: 600; letter-spacing: -.035em;
    font-variant-numeric: tabular-nums;
  }
  .hero-unit { font-size: 13px; color: var(--muted); font-weight: 500; }
  .hero-label { font-size: 10px; color: var(--muted); margin-left: 3px; }
  .rows {
    display: grid; grid-template-columns: 1fr 1fr; gap: 5px 12px;
    margin-top: 12px; padding-top: 10px; border-top: 1px solid var(--line);
    font-size: 10px;
  }
  .rows > div { display: flex; align-items: baseline; justify-content: space-between; gap: 7px; }
  .rows span { color: var(--muted); }
  .rows b { font-family: var(--mono); font-size: 10.5px; font-weight: 500; font-variant-numeric: tabular-nums; white-space: nowrap; }
  .coords {
    margin-top: 8px; font-family: var(--mono); font-size: 8.8px; color: var(--muted);
    white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
  }
  .notice {
    margin-top: 7px; font-size: 9.5px; line-height: 1.35; color: var(--muted);
    max-height: 28px; overflow: hidden;
  }
  .notice:empty { display: none; }
  .legend {
    left: 50%; transform: translateX(-50%); bottom: 18px;
    padding: 8px 14px 7px; max-width: calc(100vw - 560px);
  }
  .legend-row { display: flex; align-items: center; gap: 15px; font-size: 10px; color: #3c504f; white-space: nowrap; }
  .legend-row > span { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { display: inline-block; width: 15px; height: 2px; border-radius: 2px; background: var(--muted); }
  .swatch.amber { background: var(--amber); }
  .swatch.green { background: var(--green); }
  .swatch.blue { background: var(--blue); }
  .swatch.star {
    width: 8px; height: 8px; border-radius: 0; background: var(--base);
    clip-path: polygon(50% 0%, 61% 35%, 98% 35%, 68% 57%, 79% 94%, 50% 72%, 21% 94%, 32% 57%, 2% 35%, 39% 35%);
  }
  .swatch.boat {
    width: 7px; height: 8px; border-radius: 0; background: var(--boat);
    clip-path: polygon(50% 0%, 100% 100%, 0% 100%);
  }
  .gradbar {
    display: inline-block; width: 34px; height: 4px; border-radius: 3px;
    background: linear-gradient(90deg, #94b3bc, #4f7f8c, #224e5c);
  }
  .credit { margin-top: 5px; font-size: 8.5px; color: var(--muted); text-align: center; letter-spacing: .01em; }
  .controls {
    position: absolute; z-index: 10; right: 18px; bottom: 18px;
    display: flex; flex-direction: column; align-items: flex-end; gap: 8px;
  }
  .control-card {
    display: flex; align-items: center; gap: 11px;
    background: var(--card); border: 1px solid var(--line); border-radius: 10px;
    padding: 6px 9px; box-shadow: 0 5px 18px rgba(31,48,47,.055);
    -webkit-backdrop-filter: blur(9px); backdrop-filter: blur(9px);
  }
  .toggle {
    display: inline-flex; align-items: center; gap: 5px;
    font-size: 9.5px; color: #3c504f; cursor: pointer; user-select: none;
  }
  .toggle input { position: absolute; opacity: 0; width: 1px; height: 1px; }
  .toggle .box {
    width: 11px; height: 11px; border: 1px solid rgba(31,48,47,.35);
    border-radius: 3px; background: rgba(255,255,255,.45); display: inline-block;
    position: relative;
  }
  .toggle input:checked + .box { background: var(--ink); border-color: var(--ink); }
  .toggle input:checked + .box::after {
    content: ""; position: absolute; left: 3px; top: 1px; width: 3px; height: 6px;
    border: solid #fff; border-width: 0 1.5px 1.5px 0; transform: rotate(42deg);
  }
  .zoom { display: flex; gap: 5px; }
  .zoom button {
    width: 30px; height: 30px; padding: 0; border-radius: 8px;
    border: 1px solid var(--line); background: var(--card); color: var(--ink);
    font-family: var(--sans); font-size: 17px; line-height: 1; cursor: pointer;
    box-shadow: 0 4px 12px rgba(31,48,47,.05);
    -webkit-backdrop-filter: blur(9px); backdrop-filter: blur(9px);
  }
  .zoom button:hover { background: rgba(255,255,255,.95); }
  .zoom button:active { transform: translateY(1px); }
  #resetView { font-size: 13px; }
  .hint {
    position: absolute; z-index: 4; left: 50%; top: 16px; transform: translateX(-50%);
    font-size: 9.5px; color: var(--muted); letter-spacing: .02em;
    background: rgba(244,241,232,.8); padding: 5px 10px; border-radius: 99px;
    pointer-events: none; opacity: 0;
    animation: hint 7s ease forwards;
  }
  @keyframes hint {
    0% { opacity: 0; } 8% { opacity: 1; } 78% { opacity: 1; } 100% { opacity: 0; }
  }
  @media (max-width: 980px) {
    .legend { display: none; }
    .inset { width: 168px; }
    #inset { height: 88px; }
    .inset-credit { display: block; }
  }
  @media (max-width: 640px) {
    header { height: 52px; padding: 0 12px; }
    .brand-sub { display: none; }
    .clock { display: none; }
    .inset { width: 136px; left: 10px; top: 10px; padding: 7px 8px 6px; }
    .inset-head span { display: none; }
    #inset { height: 72px; }
    .telemetry { left: 10px; bottom: 10px; width: 195px; padding: 10px 11px 9px; }
    .hero-value { font-size: 27px; }
    .coords { display: none; }
    .controls { right: 10px; bottom: 10px; }
    .control-card { gap: 7px; padding: 5px 7px; }
    .hint { display: none; }
  }
</style>
</head>
<body>
<header>
  <div class="brand">
    <span class="brand-mark" aria-hidden="true">&#9678;</span>
    <div>
      <div class="brand-title">Retour à la base</div>
      <div class="brand-sub">Lac de Guerlédan &middot; repère local</div>
    </div>
  </div>
  <div class="topbar-right">
    <div id="statusPill" class="pill"><span class="pill-dot"></span><span id="statusText">SIMULATION</span></div>
    <div id="clock" class="clock">--:--:--</div>
  </div>
</header>
<main id="stage">
  <canvas id="map"></canvas>

  <div class="card inset" id="insetCard" title="Cliquer pour afficher tout le lac">
    <div class="inset-head">Lac de Guerlédan <span>vue d'ensemble</span></div>
    <canvas id="inset"></canvas>
    <div class="inset-credit">© OpenStreetMap (ODbL)</div>
  </div>

  <div class="card telemetry">
    <div class="hero">
      <span id="dist" class="hero-value">&mdash;</span>
      <span class="hero-unit">m</span>
      <span class="hero-label">de la base</span>
    </div>
    <div class="rows">
      <div><span>X</span><b id="posX">&mdash;</b></div>
      <div><span>Y</span><b id="posY">&mdash;</b></div>
      <div><span>Gradient</span><b id="grad">&mdash;</b></div>
      <div><span>MAJ</span><b id="age">&mdash;</b></div>
    </div>
    <div id="latlon" class="coords">&mdash;</div>
    <div id="notice" class="notice"></div>
  </div>

  <div class="card legend">
    <div class="legend-row">
      <span><i class="swatch amber"></i>Couloir</span>
      <span><i class="swatch green"></i>Bordure G.</span>
      <span><i class="swatch blue"></i>Bordure D.</span>
      <span><i class="swatch star"></i>Base</span>
      <span><i class="swatch boat"></i>Bateau</span>
      <span><i class="gradbar"></i>Gradient</span>
    </div>
    <div class="credit">Fond de lac &copy; OpenStreetMap (ODbL)</div>
  </div>

  <div class="controls">
    <div class="control-card">
      <label class="toggle"><input type="checkbox" id="showField" checked><span class="box"></span>Champ</label>
      <label class="toggle"><input type="checkbox" id="showTrail" checked><span class="box"></span>Trace</label>
    </div>
    <div class="zoom">
      <button id="zoomIn" type="button" title="Zoomer (+)" aria-label="Zoomer">+</button>
      <button id="zoomOut" type="button" title="Dézoomer (-)" aria-label="Dézoomer">&minus;</button>
      <button id="resetView" type="button" title="Voir tous les points (0)" aria-label="Voir tous les points">&#8962;</button>
    </div>
  </div>

  <div class="hint">Molette pour zoomer &middot; glisser pour déplacer</div>
</main>

<script id="static-data" type="application/json">__STATIC_JSON__</script>
<script>
(function () {
  'use strict';

  var DATA = JSON.parse(document.getElementById('static-data').textContent);
  var canvas = document.getElementById('map');
  var ctx = canvas.getContext('2d');
  var inset = document.getElementById('inset');
  var ictx = inset.getContext('2d');
  var staticLayer = document.createElement('canvas');
  var stCtx = staticLayer.getContext('2d');

  var W = 1, H = 1, DPR = 1;
  var IW = 1, IH = 1, IDPR = 1;
  var view = { cx: 0, cy: 0, scale: 1 };
  var fitted = false;
  var dirty = true;
  var state = null;
  var smooth = null;
  var showField = true;
  var showTrail = true;
  var lastStateAt = 0;
  var dragging = null;
  var pointers = {};
  var pinch = null;

  var COL = {
    land: '#f4f1e8',
    water: '#cfe4e7',
    shore: '#78989f',
    grid: 'rgba(31,48,47,0.055)',
    amber: '#bf7a21',
    green: '#2f7d5a',
    blue: '#35669b',
    base: '#a53d26',
    boat: '#d4552f',
    field: '#4d8093',
    ink: '#1f302f',
    muted: '#70807f'
  };
  var CARDINALS = ['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSO','SO','OSO','O','ONO','NO','NNO'];

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }
  function sx(x) { return W / 2 + (x - view.cx) * view.scale; }
  function sy(y) { return H / 2 - (y - view.cy) * view.scale; }
  function wx(px) { return view.cx + (px - W / 2) / view.scale; }
  function wy(py) { return view.cy - (py - H / 2) / view.scale; }
  function viewXMin() { return wx(0); }
  function viewXMax() { return wx(W); }
  function viewYMin() { return wy(H); }
  function viewYMax() { return wy(0); }

  function niceStep(target) {
    var steps = [1,2,5,10,20,25,50,100,200,250,500,1000,2000,2500,5000,10000];
    for (var i = 0; i < steps.length; i++) if (steps[i] >= target) return steps[i];
    return 10000;
  }

  function angleLerp(a, b, t) {
    var d = (b - a + Math.PI) % (2 * Math.PI);
    if (d < 0) d += 2 * Math.PI;
    d -= Math.PI;
    return a + d * t;
  }

  function setupCanvas(c) {
    var r = c.getBoundingClientRect();
    var dpr = Math.min(window.devicePixelRatio || 1, 2);
    var w = Math.max(1, Math.round(r.width));
    var h = Math.max(1, Math.round(r.height));
    c.width = Math.round(w * dpr);
    c.height = Math.round(h * dpr);
    var g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { w: w, h: h, dpr: dpr, ctx: g };
  }

  function resize() {
    var m = setupCanvas(canvas);
    W = m.w; H = m.h; DPR = m.dpr; ctx = m.ctx;
    staticLayer.width = canvas.width;
    staticLayer.height = canvas.height;
    stCtx = staticLayer.getContext('2d');
    stCtx.setTransform(DPR, 0, 0, DPR, 0, 0);

    var n = setupCanvas(inset);
    IW = n.w; IH = n.h; IDPR = n.dpr; ictx = n.ctx;

    if (!fitted) { fitView(); fitted = true; }
    dirty = true;
    drawInset();
  }

  function fitView() {
    var b = DATA.fit;
    var narrow = W < 700;
    var padX = narrow ? 34 : Math.min(160, W * 0.13);
    var padY = narrow ? Math.min(165, H * 0.22) : Math.min(105, H * 0.13);
    var s = Math.min((W - 2 * padX) / (b.xmax - b.xmin),
                     (H - 2 * padY) / (b.ymax - b.ymin));
    view.scale = clamp(s, 0.02, 30);
    view.cx = (b.xmin + b.xmax) / 2;
    view.cy = (b.ymin + b.ymax) / 2;
    dirty = true;
  }

  function fitLake() {
    var b = DATA.lake;
    var padX = Math.min(120, W * 0.09);
    var padY = Math.min(120, H * 0.09);
    var s = Math.min((W - 2 * padX) / (b.xmax - b.xmin),
                     (H - 2 * padY) / (b.ymax - b.ymin));
    view.scale = clamp(s, 0.02, 30);
    view.cx = (b.xmin + b.xmax) / 2;
    view.cy = (b.ymin + b.ymax) / 2;
    dirty = true;
  }

  function shorePath() {
    var p = new Path2D();
    var sh = DATA.shore;
    for (var i = 0; i < sh.length; i++) {
      var X = sx(sh[i][0]), Y = sy(sh[i][1]);
      if (i === 0) p.moveTo(X, Y); else p.lineTo(X, Y);
    }
    p.closePath();
    return p;
  }

  function drawGrid(c) {
    var step = niceStep(85 / view.scale);
    var x0 = Math.floor(viewXMin() / step) * step;
    var x1 = Math.ceil(viewXMax() / step) * step;
    var y0 = Math.floor(viewYMin() / step) * step;
    var y1 = Math.ceil(viewYMax() / step) * step;
    c.save();
    c.strokeStyle = COL.grid;
    c.lineWidth = 1;
    c.beginPath();
    for (var x = x0; x <= x1 + 0.001; x += step) {
      var X = Math.round(sx(x)) + 0.5;
      c.moveTo(X, 0); c.lineTo(X, H);
    }
    for (var y = y0; y <= y1 + 0.001; y += step) {
      var Y = Math.round(sy(y)) + 0.5;
      c.moveTo(0, Y); c.lineTo(W, Y);
    }
    c.stroke();
    c.restore();
  }

  function line(points, color, width, dash) {
    stCtx.save();
    stCtx.beginPath();
    for (var i = 0; i < points.length; i++) {
      var X = sx(points[i][0]), Y = sy(points[i][1]);
      if (i === 0) stCtx.moveTo(X, Y); else stCtx.lineTo(X, Y);
    }
    stCtx.strokeStyle = 'rgba(255,255,255,0.62)';
    stCtx.lineWidth = width + 2.2;
    stCtx.setLineDash(dash || []);
    stCtx.stroke();
    stCtx.strokeStyle = color;
    stCtx.lineWidth = width;
    stCtx.stroke();
    stCtx.restore();
  }

  function drawZone(c) {
    var b = DATA.fit;
    var cx = (b.xmin + b.xmax) / 2;
    var cy = (b.ymin + b.ymax) / 2;
    var r = Math.max(9, 130 * view.scale);
    var X = sx(cx), Y = sy(cy);
    c.save();
    c.beginPath();
    c.arc(X, Y, r, 0, Math.PI * 2);
    c.fillStyle = 'rgba(191,122,33,0.08)';
    c.fill();
    c.strokeStyle = 'rgba(191,122,33,0.8)';
    c.lineWidth = 1.2;
    c.setLineDash([3, 4]);
    c.stroke();
    c.setLineDash([]);
    c.font = '600 9px "Segoe UI", system-ui, sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    c.lineWidth = 3;
    c.strokeStyle = 'rgba(244,241,232,0.94)';
    c.strokeText('zone', X, Y - r - 8);
    c.fillStyle = COL.amber;
    c.fillText('zone', X, Y - r - 8);
    c.restore();
  }

  function drawCorridor(c, detail) {
    if (!detail) {
      drawZone(c);
      return;
    }
    line(DATA.lines.couloir, COL.amber, 1.7, [6, 5]);
    line(DATA.lines.gauche, COL.green, 1.6, []);
    line(DATA.lines.droite, COL.blue, 1.6, []);
  }

  function fieldSample(x, y) {
    var f = DATA.field;
    var fx = (x - f.x0) / f.step;
    var fy = (y - f.y0) / f.step;
    if (!(fx >= 0 && fy >= 0 && fx < f.nx - 1 && fy < f.ny - 1)) return null;
    var i = Math.floor(fx), j = Math.floor(fy);
    var tx = fx - i, ty = fy - j;
    var a = f.vec[j * f.nx + i];
    var b = f.vec[j * f.nx + i + 1];
    var c = f.vec[(j + 1) * f.nx + i];
    var d = f.vec[(j + 1) * f.nx + i + 1];
    var ux = (a[0] * (1 - tx) + b[0] * tx) * (1 - ty) + (c[0] * (1 - tx) + d[0] * tx) * ty;
    var uy = (a[1] * (1 - tx) + b[1] * tx) * (1 - ty) + (c[1] * (1 - tx) + d[1] * tx) * ty;
    var lm = (a[2] * (1 - tx) + b[2] * tx) * (1 - ty) + (c[2] * (1 - tx) + d[2] * tx) * ty;
    var n = Math.sqrt(ux * ux + uy * uy);
    if (n < 1e-9) return { ux: 0, uy: -1, lm: lm };
    return { ux: ux / n, uy: uy / n, lm: lm };
  }

  function mix(a, b, t) {
    return [Math.round(a[0] + (b[0] - a[0]) * t),
            Math.round(a[1] + (b[1] - a[1]) * t),
            Math.round(a[2] + (b[2] - a[2]) * t)];
  }

  function fieldColor(lm) {
    var f = DATA.field;
    var t = clamp((lm - f.lmMin) / (f.lmMax - f.lmMin || 1), 0, 1);
    var weak = [148, 179, 188], mid = [79, 127, 140], strong = [34, 78, 92];
    var c = t < 0.5 ? mix(weak, mid, t * 2) : mix(mid, strong, (t - 0.5) * 2);
    return 'rgb(' + c[0] + ',' + c[1] + ',' + c[2] + ')';
  }

  function drawArrow(c, x, y, ux, uy, len, color, alpha) {
    var ex = x + ux * len * 0.5, ey = y + uy * len * 0.5;
    var bx = x - ux * len * 0.5, by = y - uy * len * 0.5;
    var px = -uy, py = ux;
    var hx = ux * len * 0.34, hy = uy * len * 0.34;
    var hw = len * 0.22;
    c.globalAlpha = alpha;
    c.strokeStyle = color;
    c.lineWidth = 1.08;
    c.lineCap = 'round';
    c.beginPath();
    c.moveTo(bx, by);
    c.lineTo(ex, ey);
    c.moveTo(ex - hx + px * hw, ey - hy + py * hw);
    c.lineTo(ex, ey);
    c.lineTo(ex - hx - px * hw, ey - hy - py * hw);
    c.stroke();
    c.globalAlpha = 1;
  }

  function drawField(c, path) {
    if (!showField) return;
    c.save();
    c.clip(path);
    var step = niceStep(50 / view.scale);
    var i0 = Math.ceil(viewXMin() / step);
    var i1 = Math.floor(viewXMax() / step);
    var j0 = Math.ceil(viewYMin() / step);
    var j1 = Math.floor(viewYMax() / step);
    var len = clamp(step * view.scale * 0.52, 9, 19);
    for (var j = j0; j <= j1; j++) {
      for (var i = i0; i <= i1; i++) {
        var x = i * step, y = j * step;
        var v = fieldSample(x, y);
        if (!v) continue;
        var t = clamp((v.lm - DATA.field.lmMin) / (DATA.field.lmMax - DATA.field.lmMin || 1), 0, 1);
        drawArrow(c, sx(x), sy(y), v.ux, -v.uy, len, fieldColor(v.lm), 0.45 + 0.30 * t);
      }
    }
    c.restore();
  }

  function drawStar(c, x, y, r, color) {
    c.save();
    c.beginPath();
    for (var i = 0; i < 10; i++) {
      var a = -Math.PI / 2 + i * Math.PI / 5;
      var rr = (i % 2 === 0) ? r : r * 0.43;
      var X = x + Math.cos(a) * rr;
      var Y = y + Math.sin(a) * rr;
      if (i === 0) c.moveTo(X, Y); else c.lineTo(X, Y);
    }
    c.closePath();
    c.fillStyle = color;
    c.strokeStyle = 'rgba(255,255,255,0.9)';
    c.lineWidth = 2.2;
    c.stroke();
    c.fill();
    c.restore();
  }

  function label(c, text, x, y, dx, dy, align, color) {
    c.save();
    c.font = '600 10.5px "Segoe UI", system-ui, sans-serif';
    c.textAlign = align || 'left';
    c.textBaseline = 'middle';
    c.lineWidth = 3.3;
    c.strokeStyle = 'rgba(244,241,232,0.94)';
    c.strokeText(text, x + dx, y + dy);
    c.fillStyle = color;
    c.fillText(text, x + dx, y + dy);
    c.restore();
  }

  function drawPoints(c, detail) {
    if (!detail) {
      var BX = sx(0), BY = sy(0);
      drawStar(c, BX, BY, 8.3, COL.base);
      label(c, 'BASE', BX, BY, 0, 18, 'center', COL.base);
      return;
    }
    var offsets = {
      C2: [8, -8, 'left'],
      G1: [9, 11, 'left'],
      G2: [-9, 12, 'right'],
      D1: [-9, -11, 'right'],
      D2: [9, -8, 'left']
    };
    var colors = { couloir: COL.amber, gauche: COL.green, droite: COL.blue };
    for (var i = 0; i < DATA.points.length; i++) {
      var p = DATA.points[i];
      if (p.id === 'C1') continue;
      var X = sx(p.x), Y = sy(p.y);
      var col = colors[p.group] || COL.ink;
      c.save();
      c.beginPath();
      c.arc(X, Y, 4.2, 0, Math.PI * 2);
      c.fillStyle = '#fbf9f2';
      c.strokeStyle = col;
      c.lineWidth = 2;
      c.fill();
      c.stroke();
      c.restore();
      var o = offsets[p.id] || [8, -8, 'left'];
      label(c, p.id, X, Y, o[0], o[1], o[2], col);
    }

    var BX = sx(0), BY = sy(0);
    drawStar(c, BX, BY, 8.3, COL.base);
    label(c, 'BASE', BX, BY, 0, 18, 'center', COL.base);
    label(c, 'C1', BX, BY, 14, -9, 'left', COL.amber);
  }

  function drawNorthAndScale(c) {
    c.save();
    var nx = W - 32, ny = 31;
    c.translate(nx, ny);
    c.fillStyle = COL.ink;
    c.beginPath();
    c.moveTo(0, -8); c.lineTo(3.6, 3); c.lineTo(0, 0.8); c.lineTo(-3.6, 3);
    c.closePath();
    c.fill();
    c.font = '600 9px "Segoe UI", system-ui, sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'bottom';
    c.fillText('N', 0, -11);
    c.restore();

    var lenM = niceStep(95 / view.scale);
    var lenPx = lenM * view.scale;
    var x1 = W - 72, x0 = x1 - lenPx, yy = 30;
    c.save();
    c.font = '500 9px "Segoe UI", system-ui, sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'bottom';
    c.lineWidth = 3;
    c.strokeStyle = 'rgba(244,241,232,0.92)';
    c.strokeText(lenM + ' m', (x0 + x1) / 2, yy - 5);
    c.fillStyle = COL.ink;
    c.fillText(lenM + ' m', (x0 + x1) / 2, yy - 5);
    c.strokeStyle = 'rgba(244,241,232,0.92)';
    c.lineWidth = 3.2;
    c.beginPath();
    c.moveTo(x0, yy); c.lineTo(x1, yy);
    c.moveTo(x0, yy - 4); c.lineTo(x0, yy + 4);
    c.moveTo(x1, yy - 4); c.lineTo(x1, yy + 4);
    c.stroke();
    c.strokeStyle = COL.ink;
    c.lineWidth = 1.2;
    c.stroke();
    c.restore();
  }

  function drawStatic() {
    var c = stCtx;
    c.clearRect(0, 0, W, H);
    c.fillStyle = COL.land;
    c.fillRect(0, 0, W, H);
    drawGrid(c);

    var path = shorePath();
    c.fillStyle = COL.water;
    c.fill(path);
    c.save();
    c.strokeStyle = 'rgba(255,255,255,0.48)';
    c.lineWidth = 3.4;
    c.stroke(path);
    c.restore();
    c.strokeStyle = COL.shore;
    c.lineWidth = 1.15;
    c.stroke(path);

    var detail = view.scale >= 0.5;
    drawCorridor(c, detail);
    if (detail) drawField(c, path);
    drawPoints(c, detail);
    drawNorthAndScale(c);
    dirty = false;
  }

  function drawInset() {
    if (!IW || !IH) return;
    ictx.clearRect(0, 0, IW, IH);
    ictx.fillStyle = '#ece7dc';
    ictx.fillRect(0, 0, IW, IH);

    var b = DATA.lake;
    var pad = 3;
    var s = Math.min((IW - 2 * pad) / (b.xmax - b.xmin),
                     (IH - 2 * pad) / (b.ymax - b.ymin));
    var ox = (IW - (b.xmax - b.xmin) * s) / 2;
    var oy = (IH - (b.ymax - b.ymin) * s) / 2;
    function ix(x) { return ox + (x - b.xmin) * s; }
    function iy(y) { return IH - oy - (y - b.ymin) * s; }

    var sh = DATA.shore;
    ictx.beginPath();
    for (var i = 0; i < sh.length; i++) {
      if (i === 0) ictx.moveTo(ix(sh[i][0]), iy(sh[i][1]));
      else ictx.lineTo(ix(sh[i][0]), iy(sh[i][1]));
    }
    ictx.closePath();
    ictx.fillStyle = COL.water;
    ictx.fill();
    ictx.strokeStyle = COL.shore;
    ictx.lineWidth = 0.7;
    ictx.stroke();

    var vx0 = ix(viewXMin()), vx1 = ix(viewXMax());
    var vy0 = iy(viewYMin()), vy1 = iy(viewYMax());
    var rx = Math.min(vx0, vx1), ry = Math.min(vy0, vy1);
    var rw = Math.abs(vx1 - vx0), rh = Math.abs(vy1 - vy0);
    ictx.fillStyle = 'rgba(191,122,33,0.10)';
    ictx.strokeStyle = 'rgba(191,122,33,0.72)';
    ictx.lineWidth = 0.9;
    ictx.fillRect(rx, ry, rw, rh);
    ictx.strokeRect(rx, ry, rw, rh);

    if (state && state.boat) {
      ictx.beginPath();
      ictx.arc(ix(state.boat.x), iy(state.boat.y), 2.3, 0, Math.PI * 2);
      ictx.fillStyle = COL.boat;
      ictx.fill();
      ictx.strokeStyle = '#fff';
      ictx.lineWidth = 0.8;
      ictx.stroke();
    }
  }

  function drawTrail(c) {
    if (!showTrail || view.scale < 0.5 || !state || !state.trail || state.trail.length < 2) return;
    var t = state.trail;
    c.save();
    c.beginPath();
    for (var i = 0; i < t.length; i++) {
      var X = sx(t[i][0]), Y = sy(t[i][1]);
      if (i === 0) c.moveTo(X, Y); else c.lineTo(X, Y);
    }
    c.strokeStyle = 'rgba(212,85,47,0.24)';
    c.lineWidth = 1.5;
    c.lineJoin = 'round';
    c.lineCap = 'round';
    c.stroke();
    c.restore();
  }

  function drawGradientArrow(c, x, y, ux, uy) {
    var ax = x + ux * 12, ay = y + uy * 12;
    var bx = x + ux * 34, by = y + uy * 34;
    var px = -uy, py = ux;
    c.save();
    c.strokeStyle = COL.field;
    c.lineWidth = 1.8;
    c.lineCap = 'round';
    c.globalAlpha = 0.92;
    c.beginPath();
    c.moveTo(ax, ay);
    c.lineTo(bx, by);
    c.moveTo(bx - ux * 7 + px * 4.5, by - uy * 7 + py * 4.5);
    c.lineTo(bx, by);
    c.lineTo(bx - ux * 7 - px * 4.5, by - uy * 7 - py * 4.5);
    c.stroke();
    c.restore();
  }

  function drawBoat(c) {
    if (!state || !state.boat || !smooth) return;

    var bx = sx(smooth.x), by = sy(smooth.y);
    if (view.scale < 0.5) {
      c.save();
      c.beginPath();
      c.arc(bx, by, 3.2, 0, Math.PI * 2);
      c.fillStyle = COL.boat;
      c.fill();
      c.strokeStyle = '#fff';
      c.lineWidth = 1.2;
      c.stroke();
      c.restore();
      return;
    }

    var hx = sx(0), hy = sy(0);
    var dist = Math.sqrt(Math.pow(smooth.x, 2) + Math.pow(smooth.y, 2));

    c.save();
    if (dist > 8) {
      c.setLineDash([2, 5]);
      c.strokeStyle = 'rgba(31,48,47,0.19)';
      c.lineWidth = 1;
      c.beginPath();
      c.moveTo(bx, by);
      c.lineTo(hx, hy);
      c.stroke();
    }
    c.restore();

    if (state.boat.field) {
      drawGradientArrow(c, bx, by, state.boat.field.ux, -state.boat.field.uy);
    }

    var h = smooth.h;
    var fx = Math.cos(h), fy = -Math.sin(h);
    var px = -fy, py = fx;
    var alpha = state.stale ? 0.48 : 1;

    c.save();
    c.globalAlpha = alpha;
    c.beginPath();
    c.arc(bx, by, 10.5, 0, Math.PI * 2);
    c.strokeStyle = 'rgba(255,255,255,0.88)';
    c.lineWidth = 2;
    c.stroke();

    c.beginPath();
    c.moveTo(bx + fx * 11, by + fy * 11);
    c.lineTo(bx - fx * 6 + px * 5.4, by - fy * 6 + py * 5.4);
    c.lineTo(bx - fx * 3.5, by - fy * 3.5);
    c.lineTo(bx - fx * 6 - px * 5.4, by - fy * 6 - py * 5.4);
    c.closePath();
    c.fillStyle = COL.boat;
    c.strokeStyle = '#fff';
    c.lineWidth = 1.5;
    c.fill();
    c.stroke();
    c.restore();
  }

  function render() {
    if (dirty) drawStatic();
    ctx.clearRect(0, 0, W, H);
    ctx.drawImage(staticLayer, 0, 0, W, H);
    drawTrail(ctx);
    drawBoat(ctx);
    requestAnimationFrame(render);
  }

  function smoothToBoat(boat) {
    if (!boat) return;
    if (!smooth) {
      smooth = { x: boat.x, y: boat.y, h: boat.heading || 0 };
      return;
    }
    var d = Math.sqrt(Math.pow(boat.x - smooth.x, 2) + Math.pow(boat.y - smooth.y, 2));
    if (d > 80) {
      smooth.x = boat.x; smooth.y = boat.y; smooth.h = boat.heading || 0;
      return;
    }
    smooth.x += (boat.x - smooth.x) * 0.16;
    smooth.y += (boat.y - smooth.y) * 0.16;
    smooth.h = angleLerp(smooth.h, boat.heading || 0, 0.20);
  }

  function fmt(v, digits) {
    if (v === null || v === undefined || !isFinite(v)) return '—';
    return v.toFixed(digits === undefined ? 1 : digits);
  }

  function bearingText(deg) {
    if (deg === null || deg === undefined || !isFinite(deg)) return '—';
    var d = ((deg % 360) + 360) % 360;
    var card = CARDINALS[Math.round(d / 22.5) % 16];
    return Math.round(d) + '° ' + card;
  }

  function updateDom() {
    if (!state) return;
    var pill = document.getElementById('statusPill');
    var text = document.getElementById('statusText');
    var stale = !!state.stale;
    if (stale) {
      pill.className = 'pill stale';
      text.textContent = 'GPS EN ATTENTE';
    } else if (state.mode === 'live') {
      pill.className = 'pill live';
      text.textContent = 'TEMPS RÉEL';
    } else {
      pill.className = 'pill';
      text.textContent = 'SIMULATION';
    }

    var boat = state.boat;
    if (boat) {
      document.getElementById('dist').textContent = fmt(boat.distance, 1);
      document.getElementById('posX').textContent = fmt(boat.x, 1) + ' m';
      document.getElementById('posY').textContent = fmt(boat.y, 1) + ' m';
      document.getElementById('grad').textContent = bearingText(boat.field && boat.field.bearing);
      document.getElementById('latlon').textContent = (boat.lat !== null && boat.lon !== null)
        ? fmt(boat.lat, 7) + '  ' + fmt(boat.lon, 7)
        : '—';
      var age = state.age;
      document.getElementById('age').textContent = state.mode === 'demo'
        ? 'démo'
        : ((age === null || age === undefined) ? '—' : fmt(age, 1) + ' s');
    } else {
      document.getElementById('dist').textContent = '—';
      document.getElementById('posX').textContent = '—';
      document.getElementById('posY').textContent = '—';
      document.getElementById('grad').textContent = '—';
      document.getElementById('latlon').textContent = '—';
      document.getElementById('age').textContent = '—';
    }
    var notice = document.getElementById('notice');
    notice.textContent = state.notice || '';
  }

  function applyState(s) {
    state = s;
    lastStateAt = Date.now();
    if (s.boat) smoothToBoat(s.boat);
    updateDom();
    drawInset();
  }

  function poll() {
    fetch('/api/state', { cache: 'no-store' })
      .then(function (r) { if (!r.ok) throw new Error('HTTP ' + r.status); return r.json(); })
      .then(function (s) { applyState(s); setTimeout(poll, 220); })
      .catch(function () { setTimeout(poll, 900); });
  }

  function tickClock() {
    var d = new Date();
    function p(n) { return n < 10 ? '0' + n : '' + n; }
    document.getElementById('clock').textContent = p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds());
  }

  function zoomAt(factor, px, py) {
    var x = wx(px), y = wy(py);
    view.scale = clamp(view.scale * factor, 0.02, 40);
    view.cx = x - (px - W / 2) / view.scale;
    view.cy = y + (py - H / 2) / view.scale;
    dirty = true;
  }

  canvas.addEventListener('wheel', function (e) {
    e.preventDefault();
    var r = canvas.getBoundingClientRect();
    var factor = Math.exp(-e.deltaY * 0.0015);
    zoomAt(factor, e.clientX - r.left, e.clientY - r.top);
  }, { passive: false });

  canvas.addEventListener('pointerdown', function (e) {
    if (e.pointerType === 'mouse' && e.button !== 0) return;
    canvas.setPointerCapture(e.pointerId);
    pointers[e.pointerId] = { x: e.clientX, y: e.clientY };
    var ids = Object.keys(pointers);
    if (ids.length === 1) {
      dragging = { x: e.clientX, y: e.clientY };
      canvas.classList.add('dragging');
    } else if (ids.length === 2) {
      var a = pointers[ids[0]], b = pointers[ids[1]];
      var mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
      pinch = {
        dist: Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2)),
        scale: view.scale,
        wx: wx(mx - canvas.getBoundingClientRect().left),
        wy: wy(my - canvas.getBoundingClientRect().top),
        mx: mx - canvas.getBoundingClientRect().left,
        my: my - canvas.getBoundingClientRect().top
      };
      dragging = null;
    }
  });

  canvas.addEventListener('pointermove', function (e) {
    if (!pointers[e.pointerId]) return;
    pointers[e.pointerId].x = e.clientX;
    pointers[e.pointerId].y = e.clientY;
    var ids = Object.keys(pointers);
    var r = canvas.getBoundingClientRect();

    if (ids.length === 2 && pinch) {
      var a = pointers[ids[0]], b = pointers[ids[1]];
      var dist = Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
      var mx = (a.x + b.x) / 2 - r.left;
      var my = (a.y + b.y) / 2 - r.top;
      view.scale = clamp(pinch.scale * (dist / (pinch.dist || 1)), 0.02, 40);
      view.cx = pinch.wx - (mx - W / 2) / view.scale;
      view.cy = pinch.wy + (my - H / 2) / view.scale;
      dirty = true;
      return;
    }

    if (dragging) {
      view.cx -= (e.clientX - dragging.x) / view.scale;
      view.cy += (e.clientY - dragging.y) / view.scale;
      dragging.x = e.clientX;
      dragging.y = e.clientY;
      dirty = true;
    }
  });

  function endPointer(e) {
    delete pointers[e.pointerId];
    var ids = Object.keys(pointers);
    if (ids.length === 0) {
      dragging = null;
      pinch = null;
      canvas.classList.remove('dragging');
    } else if (ids.length === 1) {
      dragging = { x: pointers[ids[0]].x, y: pointers[ids[0]].y };
      pinch = null;
    }
  }
  canvas.addEventListener('pointerup', endPointer);
  canvas.addEventListener('pointercancel', endPointer);

  canvas.addEventListener('dblclick', function () { fitView(); });

  document.getElementById('zoomIn').addEventListener('click', function () { zoomAt(1.4, W / 2, H / 2); });
  document.getElementById('zoomOut').addEventListener('click', function () { zoomAt(1 / 1.4, W / 2, H / 2); });
  document.getElementById('resetView').addEventListener('click', function () { fitView(); });
  document.getElementById('insetCard').addEventListener('click', function () { fitLake(); });
  document.getElementById('showField').addEventListener('change', function (e) {
    showField = e.target.checked; dirty = true;
  });
  document.getElementById('showTrail').addEventListener('change', function (e) {
    showTrail = e.target.checked;
  });

  window.addEventListener('keydown', function (e) {
    if (e.target && e.target.tagName === 'INPUT') return;
    if (e.key === '+' || e.key === '=') zoomAt(1.35, W / 2, H / 2);
    else if (e.key === '-') zoomAt(1 / 1.35, W / 2, H / 2);
    else if (e.key === '0') fitView();
    else if (e.key === 'l' || e.key === 'L') fitLake();
    else if (e.key === 'f' || e.key === 'F') {
      var f = document.getElementById('showField');
      f.checked = !f.checked; showField = f.checked; dirty = true;
    } else if (e.key === 't' || e.key === 'T') {
      var t = document.getElementById('showTrail');
      t.checked = !t.checked; showTrail = t.checked;
    }
  });

  var resizeTimer = null;
  window.addEventListener('resize', function () {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(resize, 100);
  });

  document.getElementById('statusText').textContent = DATA.mode === 'live' ? 'TEMPS RÉEL' : 'SIMULATION';
  if (DATA.mode === 'live') document.getElementById('statusPill').className = 'pill live';

  resize();
  requestAnimationFrame(render);
  tickClock();
  setInterval(tickClock, 1000);
  poll();
})();
</script>
</body>
</html>
"""


def _installer_stub(nom, **attributs):
    module = types.ModuleType(nom)
    for cle, valeur in attributs.items():
        setattr(module, cle, valeur)
    sys.modules[nom] = module


def charger_environnement():
    """Importe Environnement sans exiger le materiel (smbus/serial)."""
    if str(BASE_DIR) not in sys.path:
        sys.path.insert(0, str(BASE_DIR))
    if str(DRIVERS_DIR) not in sys.path:
        sys.path.insert(0, str(DRIVERS_DIR))

    try:
        import smbus  # noqa: F401
    except Exception:
        class _SMBusIndisponible(object):
            def __init__(self, *args, **kwargs):
                raise RuntimeError("smbus indisponible : mode sans materiel")

        _installer_stub("smbus", SMBus=_SMBusIndisponible)

    try:
        import serial  # noqa: F401
    except Exception:
        _installer_stub("serial", Serial=object)

    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        import environnementclass
    return environnementclass.Environnement()


def charger_rivage(env):
    """Convertit le rivage OSM (lat, lon) dans le repere local de la base."""
    if str(DRIVERS_DIR) not in sys.path:
        sys.path.insert(0, str(DRIVERS_DIR))
    import ddboatclass

    convertisseur = ddboatclass.DDBoat(None, None, None)
    points = []
    for morceau in SHORE_TEXT.split():
        lat, lon = morceau.split(",")
        x, y = convertisseur.convert_cart_position(
            float(lat), float(lon), env.home_latlong, env.rotation)
        points.append([float(x), float(y)])
    return points


def cap_boussole(vx, vy):
    """Cap boussole (0 = nord, sens horaire) d'un vecteur x=est, y=nord."""
    return (90.0 - math.degrees(math.atan2(vy, vx))) % 360.0


def local_vers_latlon(x, y, origine, rotation):
    """Inverse de DDBoat.convert_cart_position (approximation locale)."""
    lat0 = float(origine[0])
    lon0 = float(origine[1])
    dx = x * math.cos(rotation) - y * math.sin(rotation)
    dy = x * math.sin(rotation) + y * math.cos(rotation)
    lat = lat0 + math.degrees(dy / R_TERRE)
    lon = lon0 + math.degrees(dx / (R_TERRE * math.cos(math.radians(lat0))))
    return lat, lon


def _arrondi(valeur, chiffres=3):
    return round(float(valeur), chiffres)


def construire_statique(env):
    lignes = {
        "couloir": [[float(v) for v in p] for p in env.couloir],
        "gauche": [[float(v) for v in p] for p in env.bordure_gauche],
        "droite": [[float(v) for v in p] for p in env.bordure_droite],
    }
    points_meta = [
        ("C1", "couloir", lignes["couloir"][0]),
        ("C2", "couloir", lignes["couloir"][1]),
        ("G1", "gauche", lignes["gauche"][0]),
        ("G2", "gauche", lignes["gauche"][1]),
        ("D1", "droite", lignes["droite"][0]),
        ("D2", "droite", lignes["droite"][1]),
    ]
    points = []
    for identifiant, groupe, position in points_meta:
        points.append({
            "id": identifiant,
            "group": groupe,
            "x": _arrondi(position[0], 2),
            "y": _arrondi(position[1], 2),
        })

    xs = [p["x"] for p in points] + [0.0]
    ys = [p["y"] for p in points] + [0.0]
    marge = 55.0
    fit = {
        "xmin": min(xs) - marge,
        "xmax": max(xs) + marge,
        "ymin": min(ys) - marge,
        "ymax": max(ys) + marge,
    }

    rivage = charger_rivage(env)
    lake = {
        "xmin": min(p[0] for p in rivage),
        "xmax": max(p[0] for p in rivage),
        "ymin": min(p[1] for p in rivage),
        "ymax": max(p[1] for p in rivage),
    }

    # Champ pre-calcule sur une grille reguliere (interpolation cote client).
    pas = 20.0
    x0, y0 = -420.0, -360.0
    nx = int(round(820.0 / pas)) + 1
    ny = int(round(720.0 / pas)) + 1
    vecteurs = []
    lm_min = None
    lm_max = None
    for j in range(ny):
        y = y0 + j * pas
        for i in range(nx):
            x = x0 + i * pas
            vx, vy = env.direction(x, y)
            vx = float(vx)
            vy = float(vy)
            norme = math.hypot(vx, vy)
            if norme < 1e-12:
                ux, uy = 0.0, -1.0
            else:
                ux, uy = vx / norme, vy / norme
            lm = math.log10(norme + 1.0)
            if lm_min is None or lm < lm_min:
                lm_min = lm
            if lm_max is None or lm > lm_max:
                lm_max = lm
            vecteurs.append([_arrondi(ux, 3), _arrondi(uy, 3), _arrondi(lm, 3)])

    static = {
        "version": 1,
        "points": points,
        "lines": lignes,
        "home": {
            "x": 0.0,
            "y": 0.0,
            "lat": float(env.home_latlong[0]),
            "lon": float(env.home_latlong[1]),
        },
        "rotation": float(env.rotation),
        "fit": fit,
        "lake": lake,
        "shore": rivage,
        "field": {
            "x0": x0,
            "y0": y0,
            "step": pas,
            "nx": nx,
            "ny": ny,
            "lmMin": _arrondi(lm_min, 3),
            "lmMax": _arrondi(lm_max, 3),
            "vec": vecteurs,
        },
        "attribution": "Fond de lac (c) OpenStreetMap contributors (ODbL)",
    }
    return static


class DemoBoat(object):
    """Trajectoire simulee : le bateau suit le gradient depuis le couloir."""

    def __init__(self, env):
        self.env = env
        self.lock = threading.Lock()
        self.stop = threading.Event()
        c0 = [float(v) for v in env.couloir[0]]
        c1 = [float(v) for v in env.couloir[1]]
        self.start = [c0[0] + (c1[0] - c0[0]) * 0.58,
                      c0[1] + (c1[1] - c0[1]) * 0.58]
        self.x = self.start[0]
        self.y = self.start[1]
        self.heading = 0.0
        self.speed = 2.2
        self.phase = "route"
        self.phase_time = time.monotonic()
        self.run_start = self.phase_time
        self.trail = deque(maxlen=420)
        self.trail.append([self.x, self.y])
        vx, vy = env.direction(self.x, self.y)
        self.heading = math.atan2(float(vy), float(vx))
        self.thread = threading.Thread(target=self._run, name="demo-bateau")
        self.thread.daemon = True
        self.thread.start()

    def _reset_locked(self):
        self.x = self.start[0]
        self.y = self.start[1]
        vx, vy = self.env.direction(self.x, self.y)
        self.heading = math.atan2(float(vy), float(vx))
        self.phase = "route"
        self.phase_time = time.monotonic()
        self.run_start = self.phase_time
        self.trail.clear()
        self.trail.append([self.x, self.y])

    def _run(self):
        precedent = time.monotonic()
        while not self.stop.wait(0.02):
            maintenant = time.monotonic()
            dt = min(0.10, max(0.0, maintenant - precedent))
            precedent = maintenant
            with self.lock:
                if self.phase == "route":
                    vx, vy = self.env.direction(self.x, self.y)
                    norme = math.hypot(float(vx), float(vy))
                    if norme < 1e-12:
                        ux, uy = 0.0, -1.0
                    else:
                        ux, uy = float(vx) / norme, float(vy) / norme
                    cible = math.atan2(uy, ux)
                    ecart = (cible - self.heading + math.pi) % (2 * math.pi) - math.pi
                    self.heading = (self.heading + ecart * min(1.0, dt * 9.0)) % (2 * math.pi)
                    self.x += ux * self.speed * dt
                    self.y += uy * self.speed * dt
                    if (not self.trail or
                            math.hypot(self.x - self.trail[-1][0],
                                       self.y - self.trail[-1][1]) >= 0.45):
                        self.trail.append([self.x, self.y])
                    if math.hypot(self.x, self.y) <= 1.6:
                        self.phase = "arrivee"
                        self.phase_time = maintenant
                    elif maintenant - self.run_start > 120.0:
                        self._reset_locked()
                elif self.phase == "arrivee":
                    if maintenant - self.phase_time > 1.8:
                        self._reset_locked()

    def get_state(self):
        with self.lock:
            x, y = self.x, self.y
            heading = self.heading
            trail = [[_arrondi(px, 1), _arrondi(py, 1)] for px, py in self.trail]
        lat, lon = local_vers_latlon(x, y, self.env.home_latlong, self.env.rotation)
        return {
            "x": x,
            "y": y,
            "lat": lat,
            "lon": lon,
            "heading": heading,
            "speed": self.speed,
            "heading_source": "gradient",
            "trail": trail,
            "age": 0.0,
            "stale": False,
        }

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)


class LiveBoat(object):
    """Lecture GPS : position convertie dans le repere local de la base."""

    def __init__(self, env, gps_index=0):
        self.env = env
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.x = None
        self.y = None
        self.lat = None
        self.lon = None
        self.course = None
        self.speed = 0.0
        self.last_fix = None
        self.trail = deque(maxlen=500)
        self.error = None
        self.notice = "GPS actif"
        self.gps = None
        self.thread = None

        if str(BASE_DIR) not in sys.path:
            sys.path.insert(0, str(BASE_DIR))
        if str(DRIVERS_DIR) not in sys.path:
            sys.path.insert(0, str(DRIVERS_DIR))
        from gps_driver_v2 import GpsIO
        import ddboatclass
        from mesure_point_gps import checksum_valide, parser_gll

        self._checksum_valide = checksum_valide
        self._parser_gll = parser_gll
        self.calcul = ddboatclass.DDBoat(None, None, None)
        self.gps = GpsIO(tty_dev=gps_index)
        self.thread = threading.Thread(target=self._lire_gps, name="gps-bateau")
        self.thread.daemon = True
        self.thread.start()

    def _lire_gps(self):
        erreurs = 0
        while not self.stop.is_set():
            try:
                recu, trame = self.gps.read_non_blocking(timeout=0.2)
            except Exception as exc:
                erreurs += 1
                with self.lock:
                    self.error = "Erreur GPS : {}".format(exc)
                if erreurs == 1:
                    print("Erreur de lecture GPS : {}".format(exc))
                if erreurs >= 5:
                    break
                time.sleep(0.5)
                continue

            if not recu or not trame:
                continue
            trame = trame.strip()
            if not trame.startswith("$"):
                continue
            entete = trame.split(",", 1)[0]
            if not entete.endswith("GLL"):
                continue
            if "*" in trame and not self._checksum_valide(trame):
                continue
            position = self._parser_gll(trame)
            if position is None:
                continue
            lat, lon = position
            try:
                x, y = self.calcul.convert_cart_position(
                    lat, lon, self.env.home_latlong, self.env.rotation)
            except Exception as exc:
                with self.lock:
                    self.error = "Conversion GPS : {}".format(exc)
                continue

            maintenant = time.monotonic()
            with self.lock:
                if self.last_fix is not None:
                    dt = maintenant - self.last_fix
                    dx = float(x) - self.x
                    dy = float(y) - self.y
                    distance = math.hypot(dx, dy)
                    if dt > 0.02:
                        self.speed = distance / dt
                    if distance > 0.6:
                        self.course = math.atan2(dy, dx)
                    if (not self.trail or
                            math.hypot(float(x) - self.trail[-1][0],
                                       float(y) - self.trail[-1][1]) >= 0.7):
                        self.trail.append([float(x), float(y)])
                else:
                    self.trail.append([float(x), float(y)])
                self.x = float(x)
                self.y = float(y)
                self.lat = float(lat)
                self.lon = float(lon)
                self.last_fix = maintenant
                self.error = None
                self.notice = "GPS actif"
                erreurs = 0

    def get_state(self):
        maintenant = time.monotonic()
        with self.lock:
            if self.last_fix is None:
                return {
                    "boat": None,
                    "trail": [],
                    "age": None,
                    "stale": True,
                    "notice": self.error or "En attente d'un fix GPS",
                }
            age = maintenant - self.last_fix
            heading = self.course if self.course is not None else 0.0
            etat = {
                "x": self.x,
                "y": self.y,
                "lat": self.lat,
                "lon": self.lon,
                "heading": heading,
                "speed": self.speed,
                "heading_source": "gps",
                "trail": [[_arrondi(px, 1), _arrondi(py, 1)] for px, py in self.trail],
                "age": age,
                "stale": age > 3.0,
            }
            notice = self.notice
            if self.error:
                notice = self.error
            etat["notice"] = notice
            return etat

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1.5)
        if self.gps is not None:
            try:
                self.gps.close()
            except Exception:
                pass


class MissionBoat(object):
    """Telemetrie UDP envoyee par mission_sortie_retour.py.

    La mission possede le GPS, les moteurs et l'IMU ; l'IHM se contente
    d'ecouter. Cela evite d'ouvrir le GPS deux fois.
    """

    PHASES = {
        "INIT": "Initialisation",
        "ATTENTE_GPS": "Attente GPS",
        "SORTIE": "Sortie de zone",
        "HORS_LIMITES": "Hors limites detecte",
        "TOUR": "Tour complet",
        "RETOUR": "Retour a la base",
        "ARRIVEE": "Base atteinte",
        "ECHEC": "Echec",
    }

    def __init__(self, env, port=5006):
        self.env = env
        self.port = port
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.paquet = None
        self.recu = None
        self.trail = deque(maxlen=800)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.thread = threading.Thread(target=self._ecouter, name="telemetrie-mission")
        self.thread.daemon = True
        self.thread.start()

    def _ecouter(self):
        while not self.stop.is_set():
            try:
                donnees, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                paquet = json.loads(donnees.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(paquet, dict) or paquet.get("source") != "ddboat-mission":
                continue
            with self.lock:
                self.paquet = paquet
                self.recu = time.monotonic()
                x, y = paquet.get("x"), paquet.get("y")
                if x is not None and y is not None and (
                        not self.trail or
                        math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= 0.7):
                    self.trail.append([float(x), float(y)])

    def _texte(self, p):
        morceaux = [self.PHASES.get(p.get("phase"), p.get("phase") or "?")]
        if p.get("dans_zone") is not None:
            morceaux.append("dans la zone" if p["dans_zone"] else "HORS LIMITES")
        if p.get("cap") is not None:
            cap = "cap {:.0f}°".format(p["cap"])
            if p.get("cap_cible") is not None:
                cap += " → {:.0f}°".format(p["cap_cible"])
            morceaux.append(cap)
        if p.get("moteurs"):
            morceaux.append("moteurs {}/{}".format(p["moteurs"][0], p["moteurs"][1]))
        if p.get("message"):
            morceaux.append(p["message"])
        return " · ".join(morceaux)

    def get_state(self):
        maintenant = time.monotonic()
        with self.lock:
            if self.paquet is None:
                return {
                    "boat": None,
                    "trail": [],
                    "age": None,
                    "stale": True,
                    "notice": "En attente de la mission (UDP {})".format(self.port),
                }
            p = dict(self.paquet)
            age = maintenant - self.recu
            trail = [[_arrondi(px, 1), _arrondi(py, 1)] for px, py in self.trail]

        if p.get("fix_age") is not None:
            age += float(p["fix_age"])
        notice = self._texte(p)
        if p.get("x") is None:
            return {"boat": None, "trail": trail, "age": None, "stale": True,
                    "notice": notice}
        return {
            "x": float(p["x"]),
            "y": float(p["y"]),
            "lat": p.get("lat"),
            "lon": p.get("lon"),
            "heading": p.get("heading") or 0.0,
            "speed": 0.0,
            "heading_source": "compas",
            "trail": trail,
            "age": age,
            "stale": age > 3.0,
            "notice": notice,
        }

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.5)
        self.sock.close()


class AppState(object):
    def __init__(self, env, bateau, mode, notice):
        self.env = env
        self.bateau = bateau
        self.mode = mode
        self.notice = notice

    def snapshot(self):
        etat = self.bateau.get_state()
        etat["mode"] = self.mode
        if self.notice and not etat.get("notice"):
            etat["notice"] = self.notice
        bateau = etat.get("boat")
        if bateau is None and etat.get("x") is not None:
            bateau = {
                "x": etat.pop("x"),
                "y": etat.pop("y"),
                "lat": etat.pop("lat"),
                "lon": etat.pop("lon"),
                "heading": etat.pop("heading"),
                "speed": etat.pop("speed"),
                "heading_source": etat.pop("heading_source"),
            }
            etat["boat"] = bateau
        if bateau is not None:
            x = bateau["x"]
            y = bateau["y"]
            vx, vy = self.env.direction(x, y)
            vx = float(vx)
            vy = float(vy)
            norme = math.hypot(vx, vy)
            if norme < 1e-12:
                ux, uy = 0.0, -1.0
            else:
                ux, uy = vx / norme, vy / norme
            bateau["distance"] = math.hypot(x, y)
            bateau["field"] = {
                "ux": _arrondi(ux, 4),
                "uy": _arrondi(uy, 4),
                "bearing": _arrondi(cap_boussole(vx, vy), 1),
                "magnitude": _arrondi(norme, 3),
            }
        etat["server_time"] = time.time()
        return etat


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    page = b""
    app_state = None

    def do_GET(self):
        route = urlparse(self.path).path
        if route == "/" or route == "/index.html":
            self._envoyer(200, "text/html; charset=utf-8", self.page)
        elif route == "/api/state":
            try:
                corps = json.dumps(self.app_state.snapshot(),
                                   ensure_ascii=False,
                                   separators=(",", ":")).encode("utf-8")
            except Exception as exc:
                corps = json.dumps({"error": str(exc)}).encode("utf-8")
                self._envoyer(500, "application/json; charset=utf-8", corps)
                return
            self._envoyer(200, "application/json; charset=utf-8", corps)
        elif route == "/favicon.ico":
            self.send_response(204)
            self.send_header("Content-Length", "0")
            self.end_headers()
        else:
            self._envoyer(404, "text/plain; charset=utf-8", b"not found")

    def _envoyer(self, code, content_type, corps):
        try:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(corps)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(corps)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt, *args):
        pass


class Serveur(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def adresse_locale():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        adresse = s.getsockname()[0]
        s.close()
        return adresse
    except Exception:
        return None


def parser_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="IHM de visualisation du Lac de Guerledan et du retour a la base.")
    parser.add_argument("--live", action="store_true",
                        help="lire la position GPS reelle au lieu de la simulation")
    parser.add_argument("--gps", type=int, choices=(0, 1, 2), default=0,
                        help="peripherique GPS : /dev/ttyGPS0, 1 ou 2 (defaut : 0)")
    parser.add_argument("--mission", action="store_true",
                        help="afficher la telemetrie de mission_sortie_retour.py")
    parser.add_argument("--udp-port", type=int, default=5006,
                        help="port UDP de la telemetrie de mission (defaut : 5006)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="adresse d'ecoute (defaut : 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000,
                        help="port HTTP (defaut : 8000)")
    parser.add_argument("--no-browser", action="store_true",
                        help="ne pas ouvrir automatiquement le navigateur")
    return parser.parse_args(argv)


def main(argv=None):
    args = parser_arguments(argv)

    try:
        env = charger_environnement()
    except Exception as exc:
        print("Impossible de charger Environnement : {}".format(exc))
        return 1

    mode = "demo"
    notice = "Simulation : trajectoire le long du gradient"
    bateau = None

    if args.mission:
        try:
            bateau = MissionBoat(env, port=args.udp_port)
            mode = "live"
            notice = "Telemetrie de mission"
            print("Ecoute de la telemetrie de mission sur le port UDP {}.".format(args.udp_port))
        except OSError as exc:
            print("Impossible d'ouvrir le port UDP {} : {}".format(args.udp_port, exc))
            return 1
    elif args.live:
        try:
            bateau = LiveBoat(env, gps_index=args.gps)
            mode = "live"
            notice = "Position GPS en direct"
            print("GPS ouvert sur /dev/ttyGPS{}.".format(args.gps))
        except Exception as exc:
            print("Mode GPS indisponible : {}".format(exc))
            print("Repli sur la simulation.")
            notice = "GPS indisponible — simulation"

    if bateau is None:
        bateau = DemoBoat(env)

    app_state = AppState(env, bateau, mode, notice)
    static = construire_statique(env)
    static["mode"] = mode
    static_json = json.dumps(static, ensure_ascii=False, separators=(",", ":"))
    page = PAGE_TEMPLATE.replace("__STATIC_JSON__", static_json)

    Handler.page = page.encode("utf-8")
    Handler.app_state = app_state

    try:
        serveur = Serveur((args.host, args.port), Handler)
    except OSError as exc:
        print("Impossible d'ouvrir le port {} : {}".format(args.port, exc))
        bateau.close()
        return 1

    hote = "127.0.0.1" if args.host in ("0.0.0.0", "::") else args.host
    url = "http://{}:{}/".format(hote, args.port)

    print("")
    print("IHM prete : {}".format(url))
    if args.host == "0.0.0.0":
        locale = adresse_locale()
        if locale:
            print("Depuis le reseau local : http://{}:{}/".format(locale, args.port))
    print("Mode : {}".format("GPS temps reel" if mode == "live" else "simulation"))
    print("Ctrl+C pour arreter.")
    print("")
    sys.stdout.flush()

    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        serveur.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nArret de l'IHM.")
    finally:
        serveur.server_close()
        try:
            bateau.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
