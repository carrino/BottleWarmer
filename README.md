# BottleWarmer

Simple code for arduino to warm stuff to body temp.  Most bottle warmers use steam which seems overly hot to me.  I wrote this up to make an existing bottle warmer into a room temp sous vide.

## Items
* 10k thermistor ($2) - https://www.adafruit.com/products/372 or https://www.amazon.com/dp/B0185P4MVO
* led buttom ($2) - https://www.adafruit.com/products/1440
* 1K resistor for button led (otherwise you will burn it out)
* 5v trinket ($7) - https://www.adafruit.com/products/1501
* relay ($2) - https://www.amazon.com/gp/product/B00XT0OSUQ
* cheap steam bottle warmer ($12) - https://www.amazon.com/gp/product/B00005BXKM

I went with 5V trinket because the relay was 5V and it made the logic simplier.  The advantage of the 3.3V trinket is the noise level off the voltage regulator is lower, but that isn't a huge concern here because we don't need that accurate of a temp reading.

After researching this more it looks like this product already exists: https://www.amazon.com/dp/B00T8VQTGQ
At least we can make it for (slightly) cheaper.
