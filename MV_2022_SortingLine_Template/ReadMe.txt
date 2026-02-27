Kratko upustvo za setup opencv-ja, pylona i podrske za serijski port.

Sto se tice opencv-ja i kreiranja projekta na Windowsu to sam vam objasnio.
Ukratko idete na opencv.org i downloadujete opencv 4.9.0.

na ubuntu je stvar mozda malo komplikovanije jer treba da bildujete opencv:

Sledite ovaj link:
https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/

Sto se tice Pylon softvera koristimo Pylon 6.3.0.
Mozete da ga downloadujete sa www.baslerweb.com

Instalira se jednostavno i na linuxu i na wondowsu.

Sto se tice serijskog porta malo se drugacije pristupa na windowsu i na linuxu i zato imate dve varijante projekta.

Dongle koji mi koristimo je neki ineski CH341 koji se automatski instalira na windowsu i to je to. Samo ispratite u Device Manager koji je com port.

Na ubuntu situacija moze d abude malo komplikovanija.

Najpre treba da instalirate biblioteku libserial.
Evo kako:
sudo apt-get update -y
sudo apt-get install -y libserial-dev

Nakon toga problem je sto moze da se desi da ovaj cip CH341 bude proeznat kao nesto drugo. Uglavnom ovde je link na objasnjenje:
https://gist.github.com/dattasaurabh82/082d13fd61c0d06c7a358c5e605ce4fd

Glavna stvar je da morate da izbrisete nekid rajver koji ga zbunjuje a to je na sledeci nacin:
sudo apt purge brltty
sudo rm -rv /var/lib/BrlAPI/

RAD UBUNTU:
Sto se tice rada, uvek radite tako sto odradite unzip foldera koji se zove MV_Solutions.zip , priemenujete ga i otvorite vas folder u VSCODE-u.
Ona uradite
cmake .

zatim
make

I nakon toga moze da startujete debug sa F5. medjutim, nakon prvog startovanja izbacice gresku kako nema neku dozvolu. Fora je sto mora da se uradi sledece:
sudo chmod +x sudo_gdb.sh 