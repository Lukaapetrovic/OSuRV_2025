Ovaj projekat implementira kontrolu motora za sortiranje otpada koristeći Raspberry Pi, prilagođeni kernel drajver i ZeroMQ komunikaciju između džojstika (publisher) i kontrolnog čvora (subscriber).



Sistem za sortiranje otpada pomoću dve pokretne letvice koje pokreće DC motor (6V). Projekat omogućava dva režima upravljanja, manuelno i softversko:

*Izbor režima (Naizmenični prekidač)

	Manuelni mod (OFF): Letvicama se upravlja isključivo putem fizičkog trosmernog prekidača (3-way switch).

	Softverski mod (ON): Upravljanje se vrši preko dugmića na džojstiku.
	

*Softversko zaustavljanje (Limit Switches)
Pinovi na mikrokontroleru se koriste kao graničnici za bezbedno zaustavljanje letvica u softverskom modu:

	Pin 23 (Levo): Softverski zaustavlja pomeranje ulevo.
	
	Pin 24 (Desno): Softverski zaustavlja pomeranje udesno.
	
	Pin 22 (Sredina): Detektuje centralnu poziciju letvica.



*Kontrole na džojstiku:

	Taster 4  -->   Levo	=> Pomera motor do krajnjeg levog položaja (Pin 23).
	
	Taster 1  -->   Sredina	=> Postavlja motor u centralni položaj (Pin 22).
	
	Taster 2  -->   Desno	=> Pomera motor do krajnjeg desnog položaja (Pin 24).


Struktura projekta:

	Driver/gpio_ctrl: Izvorni kod kernel drajvera za kontrolu GPIO pinova.

	APP/joypad: Aplikacija koja čita ulaze sa džojstika i šalje komande.

	APP/wiper_node: Glavna logika koja upravlja motorom na osnovu stanja senzora i primljenih komandi.
	
	
	
*Podizanje drajvera (Kernel Space):

Prvo je potrebno učitati drajver koji omogućava pristup hardveru preko /dev/gpio_stream. Idite u folder drajvera: 
		cd Garbage_Sort/SW/Driver/gpio_ctrl
		make
		make start
		dmesg




*Kompajliranje aplikacija:

Vratite se u APP folder i pripremite projekat koristeći waf alat: 
		cd ../../APP/joypad
		./waf configure
		./waf build



*Pokretanje sistema:

		Terminal 1 (Publisher - Joypad):
		cd build
		./joy_pad

		Terminal 2 (Subscriber - Wiper Node):
		cd build
		./wiper_node

Link za Youtube video: https://youtube.com/shorts/IFT6f-TpfYg?si=OtBIaBamz4evVs-o
