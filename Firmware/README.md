# FiresportTimer-PIO
### Funguje to nasledovne:
- Pin change interrupt handler resi: 
  - veci co potrebuji bejt hned:
    - zapinani interruptu na pinech
    - zaznam casu
  - nastavovani flagu (veci zacinajici `TIMER_CONTROL_`) pro veci ktery pockaj do `volatile uint8_t timer_control` ty potom resi main loop metodou `timer_event()`
    - uart
    - ui

#### PCMSK (Pin change interrupt mask register) - zapina interrupty na pinech
    Hodnota v PCMSK slouzi zaroven jako status a jako validni piny pro kazdej moznej stav casomiry
