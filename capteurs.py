#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capteurs.py - capteurs Arduino embarques (lecture HTTP) et detecteurs d'alerte

    from capteurs import Capteurs, Gaz, Chaleur, Vapeur, DETECTEURS
    capteurs = Capteurs("http://IP:8080") ; capteurs.start()
    gaz = Gaz(capteurs) ; gaz.start()          # calibre 30 s puis surveille
    gaz.alerte -> True quand la lecture depasse le repos x 1.4
    Chaleur : +5 C au-dessus de l'ambiante ; Vapeur : brut > 300.

Le capteur MQ ne donne pas d'unite fiable : on mesure une VARIATION par
rapport a l'air ambiant. Laisser chauffer le capteur 3-5 min avant de lancer.

TEST, capteur par capteur
    python3 capteurs.py --serie /dev/ttyACM0     Arduino branche sur CE PC
    python3 capteurs.py IP_ROBOT                 Arduino branche au robot
    python3 capteurs.py IP_ROBOT --detecteurs    idem + detecteurs d'alerte
        Chaque valeur est affichee avec un avis (plausible / cablage suspect).
"""

import sys
import threading
import time

import requests

CLE_GAZ = "gaz"
DUREE_CALIBRATION_S = 30.0
RATIO_ALERTE = 1.4        # lecture > repos x 1.4 -> alerte
RATIO_FIN = 1.15          # lecture < repos x 1.15 -> fin d'alerte
VOTES = 3                 # lectures consecutives pour changer d'etat


class Capteurs(threading.Thread):
    """Interroge /capteurs a 5 Hz et garde la derniere lecture brute."""

    def __init__(self, url, verbeux=True):
        super().__init__(daemon=True)
        self.url = url.rstrip("/") + "/capteurs"
        self.verbeux = verbeux
        self.brut = None              # derniere mesure FRAICHE {"c": 87, "gaz": 312, ...}
        self.dernier = None           # derniere reponse du serveur, meme perimee
        self.erreur = None
        self.disponible = False
        self._arret = threading.Event()

    def run(self):
        echecs = 0
        while not self._arret.is_set():
            try:
                m = requests.get(self.url, timeout=0.5).json()
                self.dernier = m              # meme perime : utile au diagnostic
                self.erreur = None
                if m.get("perime"):
                    raise ValueError("perime")
                if not self.disponible and self.verbeux:
                    print("[capteurs] actif : %s" % {k: v for k, v in m.items()
                                                      if k not in ("age", "perime")})
                self.brut = m
                self.disponible = True
                echecs = 0
            except requests.RequestException as erreur:
                self.erreur = "serveur injoignable (%s)" % erreur.__class__.__name__
                echecs += 1
                if echecs == 10:
                    self.disponible = False
                    if self.verbeux:
                        print("[capteurs] serveur muet")
            except Exception:
                self.erreur = "mesure perimee : l'Arduino n'envoie rien au robot"
                echecs += 1
                if echecs == 10:
                    self.disponible = False
                    if self.verbeux:
                        print("[capteurs] serveur muet")
            self._arret.wait(0.2)

    def valeur(self, cle):
        return self.brut.get(cle) if (self.disponible and self.brut) else None

    def arreter(self):
        self._arret.set()


class Detecteur(threading.Thread):
    """Surveille UNE cle des capteurs et leve une alerte avec hysteresis.

    Trois modes :
      "ratio"  : alerte si lecture > repos x seuil      (gaz : variation relative)
      "delta"  : alerte si lecture > repos + seuil      (temperature : +5 C)
      "absolu" : alerte si lecture > seuil              (vapeur : brut > 300)
    "ratio" et "delta" commencent par une calibration (moyenne pendant
    duree_calibration s, robot dans l'air ambiant). VOTES lectures consecutives
    sont necessaires pour entrer ou sortir d'alerte.
    """

    CLE = ""
    NOM = ""
    MODE = "ratio"
    SEUIL_ALERTE = 1.4
    SEUIL_FIN = 1.15
    DUREE_CALIBRATION = 30.0
    TEXTE_ALERTE = ""          # pour le modele de langage
    PHRASE = ""                # ce que dit le robot ; {v} = valeur

    def __init__(self, capteurs, seuil_alerte=None, seuil_fin=None, verbeux=True,
                 sur_alerte=None, sur_fin=None, **compat):
        super().__init__(daemon=True)
        # compat : l'ancien constructeur Gaz(capteurs, ratio_alerte=...)
        if "ratio_alerte" in compat and seuil_alerte is None:
            seuil_alerte = compat["ratio_alerte"]
        self.capteurs = capteurs
        self.seuil_alerte = self.SEUIL_ALERTE if seuil_alerte is None else seuil_alerte
        self.seuil_fin = self.SEUIL_FIN if seuil_fin is None else seuil_fin
        self.verbeux = verbeux
        self.sur_alerte = sur_alerte        # callback(valeur) au passage en alerte
        self.sur_fin = sur_fin              # callback() au retour a la normale
        self.repos = None
        self.lecture = None
        self.ratio = None                   # ratio, delta ou lecture selon MODE
        self.alerte = False
        self.present = False
        self.pret = threading.Event()
        self._votes = []
        self._arret = threading.Event()

    # -- grandeur comparee au seuil --------------------------------------------

    def _indicateur(self, lecture):
        if self.MODE == "ratio":
            return lecture / self.repos if self.repos else None
        if self.MODE == "delta":
            return lecture - self.repos if self.repos is not None else None
        return lecture

    def _calibrer(self):
        if self.MODE == "absolu":
            self.repos = 0.0
            return True
        if self.verbeux:
            print("[%s] calibration %.0f s dans l'air ambiant..." % (self.NOM, self.DUREE_CALIBRATION))
        echantillons = []
        fin = time.time() + self.DUREE_CALIBRATION
        while time.time() < fin and not self._arret.is_set():
            v = self.capteurs.valeur(self.CLE)
            if v is not None:
                echantillons.append(float(v))
            self._arret.wait(0.5)
        if len(echantillons) < 5:
            print("[%s] calibration impossible (%d lectures)" % (self.NOM, len(echantillons)))
            return False
        self.repos = sum(echantillons) / len(echantillons)
        ecart = (sum((e - self.repos) ** 2 for e in echantillons) / len(echantillons)) ** 0.5
        if self.verbeux:
            borne = (self.repos * self.seuil_alerte if self.MODE == "ratio"
                     else self.repos + self.seuil_alerte)
            print("[%s] repos = %.1f (bruit +/-%.1f), alerte au-dessus de %.1f"
                  % (self.NOM, self.repos, ecart, borne))
        return True

    def run(self):
        fin_attente = time.time() + 10
        while self.capteurs.valeur(self.CLE) is None and time.time() < fin_attente:
            self._arret.wait(0.2)
        if self.capteurs.valeur(self.CLE) is None:
            if self.verbeux:
                print("[%s] aucune cle '%s' publiee par l'Arduino : detecteur inactif"
                      % (self.NOM, self.CLE))
            self.pret.set()
            return
        self.present = True
        if not self._calibrer():
            self.pret.set()
            return
        self.pret.set()

        while not self._arret.is_set():
            v = self.capteurs.valeur(self.CLE)
            if v is not None:
                self.lecture = float(v)
                self.ratio = self._indicateur(self.lecture)
                if self.ratio is not None:
                    self._votes = (self._votes + [self.ratio])[-VOTES:]
                    if not self.alerte and len(self._votes) == VOTES and \
                            all(r > self.seuil_alerte for r in self._votes):
                        self.alerte = True
                        if self.verbeux:
                            print("[%s] ALERTE : lecture %.1f (indicateur %.2f)"
                                  % (self.NOM, self.lecture, self.ratio))
                        if self.sur_alerte:
                            self.sur_alerte(self.lecture)
                    elif self.alerte and len(self._votes) == VOTES and \
                            all(r < self.seuil_fin for r in self._votes):
                        self.alerte = False
                        if self.verbeux:
                            print("[%s] retour a la normale (%.2f)" % (self.NOM, self.ratio))
                        if self.sur_fin:
                            self.sur_fin()
            self._arret.wait(0.5)

    def resume(self):
        """Texte qualitatif pour le modele de langage ; vide hors alerte."""
        return self.TEXTE_ALERTE if (self.present and self.alerte) else ""

    def phrase(self):
        return self.PHRASE.format(v=self.lecture if self.lecture is not None else 0)

    def arreter(self):
        self._arret.set()


class Gaz(Detecteur):
    CLE, NOM, MODE = "gaz", "gaz", "ratio"
    SEUIL_ALERTE, SEUIL_FIN, DUREE_CALIBRATION = RATIO_ALERTE, RATIO_FIN, DUREE_CALIBRATION_S
    TEXTE_ALERTE = "Capteur de gaz : CONCENTRATION ANORMALE detectee ici, eloigne-toi de cette zone."
    PHRASE = "Attention, je detecte du gaz"


class Chaleur(Detecteur):
    """Thermistance : alerte quand la temperature monte de SEUIL_ALERTE degres
    au-dessus de l'ambiante mesuree au demarrage (source de chaleur proche)."""
    CLE, NOM, MODE = "temp", "chaleur", "delta"
    SEUIL_ALERTE, SEUIL_FIN, DUREE_CALIBRATION = 5.0, 2.0, 20.0
    TEXTE_ALERTE = "Capteur de temperature : SOURCE DE CHALEUR proche, ne t'approche pas."
    PHRASE = "Attention, source de chaleur, {v:.0f} degres"


class Vapeur(Detecteur):
    """Steam sensor : ~0-50 a sec, plusieurs centaines mouille ou dans la vapeur."""
    CLE, NOM, MODE = "vapeur", "vapeur", "absolu"
    SEUIL_ALERTE, SEUIL_FIN, DUREE_CALIBRATION = 300.0, 150.0, 0.0
    TEXTE_ALERTE = "Capteur d'humidite : VAPEUR ou EAU detectee ici."
    PHRASE = "Attention, humidite detectee"


DETECTEURS = (Gaz, Chaleur, Vapeur)


# ---------------------------------------------------------------------------
# Test : affichage interprete de chaque capteur
# ---------------------------------------------------------------------------

def volts(adc):
    return adc * 5.0 / 1023.0


def pourcent(adc):
    return adc * 100.0 / 1023.0


def interpreter(brut):
    """Une ligne par capteur, unites reelles, et un avis de plausibilite.

    Unites honnetes : cm (ultrason), C (temperature). Gaz, vapeur et lumiere
    sont des tensions (V, % de la pleine echelle) : ppm et lux exigeraient une
    calibration par module. Le lumen n'est pas mesurable par une photoresistance.
    """
    L = []
    absent = "non detecte par l'Arduino (rien sur la broche, ou S sur le +)"

    for cle, nom in (("c", "ultrason"), ("g", "ultrason g"), ("d", "ultrason d")):
        v = brut.get(cle)
        if v is None:
            if cle == "c":
                L.append("  %-12s %s" % (nom, absent))
            continue
        if v < 0 and cle != "c":
            continue                                    # lateral non installe ou sans echo : on tait
        if v < 0:
            L.append("  %-12s > 400 cm  (pas d'echo ; main a 30 cm pour verifier)" % nom)
        else:
            L.append("  %-12s %3d cm" % (nom, v))

    if "gaz" in brut:
        v = brut["gaz"]
        avis = ("  <- tres bas : AO branche ? module alimente ?" if v < 15 else
                "  (derive normale pendant la chauffe, 3-5 min)")
        L.append("  %-12s %.2f V  (%2.0f %%)%s" % ("gaz", volts(v), pourcent(v), avis))
    else:
        L.append("  %-12s %s" % ("gaz", absent))

    if "tadc" in brut or "temp" in brut:
        if "temp" in brut:
            L.append("  %-12s %.1f C   (doit monter avec la main ; sinon INVERSER_TEMP)" % ("temperature", brut["temp"]))
        else:
            L.append("  %-12s conversion hors plage (ADC %s) : cablage ou type de capteur"
                     % ("temperature", brut.get("tadc")))
    else:
        L.append("  %-12s %s" % ("temperature", absent))

    if "vapeur" in brut:
        v = brut["vapeur"]
        avis = "  (sec)" if v < 60 else "  (humide / vapeur)"
        L.append("  %-12s %.2f V  (%2.0f %%)%s" % ("vapeur", volts(v), pourcent(v), avis))
    else:
        L.append("  %-12s %s" % ("vapeur", absent))

    if "lum" in brut:
        v = brut["lum"]
        L.append("  %-12s %.2f V  (%2.0f %%)   lux non calibres ; masquer le capteur doit la faire bouger"
                 % ("lumiere", volts(v), pourcent(v)))
    else:
        L.append("  %-12s %s" % ("lumiere", absent))
    return "\n".join(L)


def lire_serie_local(port, bauds=115200):
    """Lit l'Arduino branche sur CE PC. pyserial si present, sinon stty + fichier."""
    import subprocess
    try:
        import serial
        return serial.Serial(port, bauds, timeout=1)
    except ImportError:
        subprocess.call(["stty", "-F", port, str(bauds), "raw", "-echo", "-hupcl"])
        return open(port, "rb", buffering=0)


def analyser_trame(ligne):
    brut = {}
    for morceau in ligne.strip().split():
        cle, egal, valeur = morceau.partition("=")
        if not egal:
            continue
        try:
            brut[cle] = int(valeur)
        except ValueError:
            try:
                brut[cle] = float(valeur)
            except ValueError:
                pass
    return brut


def test_serie(port):
    print("Lecture directe de l'Arduino sur %s (Ctrl+C pour finir)\n" % port)
    flux = lire_serie_local(port)
    tampon, dernier_affichage = b"", 0.0
    try:
        while True:
            octet = flux.read(1)
            if not octet:
                print("  ... rien recu depuis 1 s : sketch televerse ? 115200 bauds ?")
                continue
            if octet != b"\n":
                tampon += octet
                continue
            brut = analyser_trame(tampon.decode("ascii", "ignore"))
            tampon = b""
            if brut and time.time() - dernier_affichage > 1.0:
                dernier_affichage = time.time()
                print("\033[2J\033[H", end="")            # efface l'ecran
                print("trame : %s\n" % brut)
                print(interpreter(brut))
    except KeyboardInterrupt:
        pass


def test_http(ip, avec_detecteurs):
    capteurs = Capteurs("http://%s:8080" % ip, verbeux=False)
    capteurs.start()
    detecteurs = []
    if avec_detecteurs:
        detecteurs = [D(capteurs) for D in DETECTEURS]
        for d in detecteurs:
            d.start()
    print("Lecture via le robot, http://%s:8080/capteurs (Ctrl+C pour finir)\n" % ip)
    try:
        while True:
            time.sleep(1)
            print("\033[2J\033[H", end="")
            if capteurs.erreur and not capteurs.disponible:
                print("ETAT : %s" % capteurs.erreur)
                if capteurs.dernier:
                    print("derniere reponse : %s" % capteurs.dernier)
                print("\nA verifier : le serveur tourne sur le robot (ssh, cat capteurs.log) ;"
                      "\n             l'Arduino est branche AU ROBOT, pas au PC.")
                continue
            m = capteurs.dernier or {}
            age = m.get("age")
            print("trame : %s   (age %.2f s)\n" % ({k: v for k, v in m.items() if k not in ("age", "perime")},
                                                    age if age is not None else -1))
            print(interpreter(m))
            for d in detecteurs:
                if d.present:
                    etat = ("calibration..." if d.repos is None and d.MODE != "absolu" else
                            "lecture %.1f  indicateur %.2f  seuil %.2f  %s" % (
                                d.lecture or 0, d.ratio or 0, d.seuil_alerte, "ALERTE" if d.alerte else ""))
                    print("  [%s] %s" % (d.NOM, etat))
    except KeyboardInterrupt:
        for d in detecteurs:
            d.arreter()
        capteurs.arreter()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="test des capteurs Arduino, un par un")
    p.add_argument("ip", nargs="?", help="IP du robot (lecture via son serveur)")
    p.add_argument("--serie", help="port de l'Arduino branche sur CE PC, ex. /dev/ttyACM0")
    p.add_argument("--detecteurs", action="store_true", help="lancer aussi les detecteurs d'alerte")
    a = p.parse_args()
    if a.serie:
        test_serie(a.serie)
    elif a.ip:
        test_http(a.ip, a.detecteurs)
    else:
        p.error("donnez l'IP du robot, ou --serie /dev/ttyACM0")
