DRON MAGENTA

Projekat ima implemetirano poketanje motora i osnovnu PID regulaciju. Dron za cilj ima kratko odvajanje od podloge i vracanje na istu, uz regulaciju oscilacija koje se javljaju prilikom letenja. 

Bezbednosne mere pre poletanja:
- Proveriti stanje propelera 
- Proveriti da su svi zavrtnji dobro zategnuti (u smeru suprotnom od smera kretanja propelera)
- Proveriti napon baterije i da li je puna
- Proveriti da je memorijska kartica uključena u ESP32
- Testirati WiFi konekciju pre poletanja


Pokretanje projekta:
    1. Build project
    2. Flash project na ESP32 preko USB-a
    3. Vracanje ESP-a na protoboard
    4. Povezivanje napajanja (baterije)
    5. Povezivanje laptopa na WiFi mrezu drona - username: DRON-magenta, password: 12345678
    6. Pokretanje Python skripte za citanje komandi motora sa tastature komandom "python3 controls.py" iz direktorijuma u kom se ona nalazi
    7. Upravljanje jacinom motora preko tastature (UP - povecaj brzinu, DOWN - smanji brzinu)
    8. Hitno gasenje motora pritiskom na dugme SPACE
    9. Klikom na dugme 'q' se izlazi iz skripte


Trenutni parametri za PID regulaciju:
    1. Kp = 1.3 - 1.5  (proporcionalna korekcija)
    2. Ki = 0.0002 - 0.0005  (eliminacija drifta)
    3. Kd = 0.3  (prigusivanje oscilacija)


Organizacija softvera:
    main.c          - Inicijalizacija sistema, WiFi AP, UDP server
    mpu.c           - MPU6050 drajver, PID kontroler, motor control task
    mpu.h           - Definicije i deklaracije funkcija
    controls.py     - Python skripta za daljinsko upravljanje