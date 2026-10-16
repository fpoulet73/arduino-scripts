AstralPool v1.3.10
SPDT1 pump: holding register 1006. SPDT4 flow: holding register 1007. Machine enabled switch (manual): 1100.
Cache, timestamps, retry behavior preserved.

Unzip into /root, then:
arduino-cli compile --fqbn esp32:esp32:esp32c3 /root/AstraPool
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32c3 /root/AstraPool

Compilation not verified here.
