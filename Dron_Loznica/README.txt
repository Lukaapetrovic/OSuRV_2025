Na projektu radili: Dusko Obradovic(RA40/2023), Sinisa Stevanovic(RA58/2023), Aleksa Jerotic(RA205/2023), Mihailo Djurdjevic(RA206/2023)
Okruzenje: Arduino IDE
Potrebne biblioteke: MPU9250_WE(Wolfgang Ewald), ESP32Servo(Kevin Harrington, John K. Bennett).
1. Prilikom pokretanja drona povezati se na Wi-Fi Dron_Kontrola(password: 12345678 ).
2. Nakon povezivanja na server uci na link: http://192.168.1.1
3. Stop dugme vraca throttle na 0 i gasi dron.
4. GAS +20 dugme povecava thorttle za 20 u skali od 1000 do 2000(na 1290 polece dron).
5. GAS -50 dugme smanjuje throttle za 50 u skali od 1000 do 2000.
6. Setpoint (Zeljene vrednosti) su nagibi koje dron treba da postigne. Ako stavimo setpoint za roll -10, to znaci da je naget za 2 stepena.
7. Roll/Pitch PID- roll i pitch dele iste parametre tako da se u sledecom sekciji ti parametri podesavaju.
8. Yaw PID - U ovoj sekciji menjamo PID koefcijente za yaw.
9. Dugme SNIMI SVE PID-ove aplouduje nove vrednosti pidova na dron.
10. Loop Timer: Vreme od paljena drona u mikro-sekundama.
