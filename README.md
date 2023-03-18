# ch32v305

## Configure
```
cmake -S . -B build
```

## Build
```
cmake --build build -j20
```

## Flash
```
~/hdd/tools/openocd-wch-link/openocd -f ~/hdd/tools/openocd-wch-link/wch-riscv.cfg -c  "init;halt;program build/ch32v305_fw.bin 0x0;reset;resume;exit"
```