import timeit
import crc8cy
import crc8

data = """In the beginning there was silence and darkness
All across the earth
Then came the wind and a hole in the sky
Thunder and lightning came crashing down
Hit the earth and split the ground
Fire burned high in the sky

From down below fire melted the stone
The ground shook and started to pound

The gods made heavy metal and they saw that is was good
They said to play it louder than Hell
We promised that we would
When losers say it's over with you know that it's a lie
The gods made heavy metal and it's never gonna die

We are the true believers
It's our turn to show the world
In the fire of heavy metal we were burned
It's more than our religion it's the only way to live
But the enemies of metal we can't forgive

Cause we believe in the power and the might
And the gods who made metal are with us tonight

The gods made heavy metal and they saw that is was good
They said to play it louder than Hell
We promised that we would
When losers say it's over with you know that it's a lie
The gods made heavy metal and it's never gonna die

We believe in the power and the might
And the gods who made metal are with us tonight

We're here tonight for heavy metal are you ready in the hall
They have chosen us and we have heard the call
Gonna tear the roof off with our sound

Crack the walls and shake the ground
Fight tonight for metal one and all

Cause we believe in the power and the might
And the gods who made metal are with us tonight

The gods made heavy metal and they saw that is was good
They said to play it louder than Hell
We promised that we would
When losers say it's over with you know that it's a lie
The gods made heavy metal and it's never gonna die
"""

print 'crc8:', timeit.timeit('crc8.crc8(data).digest()', setup="from __main__ import crc8, data", number = 1000), 'sec'
print 'crc8cy', timeit.timeit('crc8cy.crc8(data).digest()', setup="from __main__ import crc8cy, data", number = 1000), 'sec'

c1 = crc8.crc8()
c1.update(data[:100])
c1.update(data[100:])

c2 = crc8cy.crc8()
c2.update(data[:100])
c2.update(data[100:])
assert c1.digest() == c2.digest()
