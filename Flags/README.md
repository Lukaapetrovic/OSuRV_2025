# Flags (audio → flag lift)

Sistem za podizanje zastavica na osnovu klasifikacije audio snimka (muzike). PC/razvojna mašina pokreće ML model koji prepoznaje državu iz pjesme i šalje rezultat preko UDP-a ka Raspberry Pi koji upravlja stepper motorima.

## Arhitektura sistema

### 1. MFCC Feature Extraction (Reson)
- **reson/** - C++ biblioteka za računanje MFCC koeficijenata za frejm po frejm analizu pesama
- Optimizovana za real-time obradu audio signala

### 2. Machine Learning Pipeline (model/)
- **Augmentacija audio podataka** - povećava diverzitet trening seta
- **mfcc_utils.py** - pomoćne funkcije za MFCC obradu
- **calculate_mfcc.py** - računanje MFCC koeficijenata za augmentovane chunk-ove
- **labels.npy** - mapiranje klasa (spanija/srbija/jamajka)
- **train.py** - treniranje CNN modela na raw podacima
- **best_country_model.h5** - obučeni TensorFlow model
- **offline_predict.py** - predikcija pjesme iz raw_data foldera

**Napomena:** Pokušan je real-time predict, ali postoje problemi sa šumom na mikrofonu.

### 3. Hardware Control (pi/)
- **UDP server** - prima podatke od PC-a preko UDP protokola
- **I2C komunikacija** - komunicira sa Jetsonom
- **Stepper motor control** - Jetson upravlja stepper motorima povezanim na portove
- **Flag lifting** - podiže zastavice na osnovu prepoznate države

## Tok rada

1. PC učitava audio fajl i izvlači MFCC feature-e (reson biblioteka)
2. ML model (TensorFlow) predviđa državu (Španija/Srbija/Jamajka)
3. PC šalje UDP paket sa rezultatom ka Raspberry Pi
4. RPi prima poruku i prosleđuje komandu preko I2C-a ka Jetsonu
5. Jetson aktivira odgovarajući stepper motor koji podiže zastavicu

## Brzi start

### PC (Predikcija)
```bash
cd Flags/SW/Motor_Ctrl
python3 predictionUdp.py
# Izaberi .wav fajl iz raw_data/
```

### Raspberry Pi (Hardware kontrola)
```bash
cd Flags/SW/Motor_Ctrl/pi
python3 rpi_udp.py
```

## Struktura foldera

```
Flags/SW/Motor_Ctrl/
├── reson/              # C++ MFCC biblioteka
├── model/              # ML pipeline
│   ├── augment.py      # Audio augmentacija
│   ├── mfcc_utils.py   # MFCC pomoćne funkcije
│   ├── calculate_mfcc.py
│   ├── train.py        # Treniranje modela
│   ├── offline_predict.py
│   ├── best_country_model.h5
│   └── labels.npy
├── pi/                 # Raspberry Pi kod
│   ├── rpi_udp.py      # UDP server
│   └── motors.py       # I2C/motor kontrola
└── raw_data/           # Training audio fajlovi
    ├── spanija/
    ├── srbija/
    └── jamajka/
```

## Poznati problemi

- **Real-time predikcija**: Šum na mikrofonu utiče na kvalitet predikcije
- **Network setup**: IP adrese moraju biti podešene u kodu (PC ↔ RPi komunikacija)


## Linkovi

Youtube video: https://youtu.be/rRIwtviOpec?si=6fFKUAO7426B94Af