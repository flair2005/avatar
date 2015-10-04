#!/bin/bash
# by Arnaud Ramey (arnaud.a.ramey@gmail.com)

# normal
#~ bash layers2pngs.bash  svg/normal_blink_left.svg   01                          mini_eyes/normal/open &
#~ bash layers2pngs.bash  svg/normal_blink_left.svg   01,03,06,08,10,08,06,03,01  mini_eyes/normal/blink

# angry
#~ bash layers2pngs.bash  svg/angry_blink_left.svg   01                             mini_eyes/angry/open &
#~ bash layers2pngs.bash  svg/normal_angry_left.svg  01,02,03,04,05,06,07,08,09,10  mini_eyes/angry/begin &
#~ bash layers2pngs.bash  svg/angry_blink_left.svg   01,03,06,08,10,08,06,03,01     mini_eyes/angry/blink
#~ wait; for FILE in mini_eyes/angry/*.png; do convert -flop $FILE $FILE; done

# laughing
#~ bash layers2pngs.bash  svg/normal_laughing_left.svg  10                                mini_eyes/laughing/open &
#~ bash layers2pngs.bash  svg/normal_laughing_left.svg  01,02,03,04,05,06,07,08,09,10     mini_eyes/laughing/begin &
#~ bash layers2pngs.bash  svg/normal_laughing_left.svg  10,08,06,04,02,01,02,04,06,08,10  mini_eyes/laughing/blink

# sad
#~ bash layers2pngs.bash  svg/sad_blink_left.svg   01                          mini_eyes/sad/open &
#~ bash layers2pngs.bash  svg/normal_sad_left.svg  01,02,03,04,05,06,07        mini_eyes/sad/begin &
#~ bash layers2pngs.bash  svg/sad_blink_left.svg   01,03,06,08,10,08,06,03,01  mini_eyes/sad/blink
#~ wait; for FILE in mini_eyes/sad/*.png; do convert -flop $FILE $FILE; done

# sleeping
#~ bash layers2pngs.bash  svg/normal_blink_left.svg   10                             mini_eyes/sleeping/open &
#~ bash layers2pngs.bash  svg/normal_blink_left.svg   01,02,03,04,05,06,07,08,09,10  mini_eyes/sleeping/begin &
#~ bash layers2pngs.bash  svg/normal_blink_left.svg   10,09,10                       mini_eyes/sleeping/blink

# sleepy
#~ bash layers2pngs.bash  svg/normal_blink_left.svg  06                             mini_eyes/sleepy/open &
#~ bash layers2pngs.bash  svg/normal_blink_left.svg  01,02,03,04,05                 mini_eyes/sleepy/begin &
#~ bash layers2pngs.bash  svg/normal_blink_left.svg  06,07,08,09,10,10,09,08,07,06  mini_eyes/sleepy/blink

# surprised
#~ bash layers2pngs.bash  svg/surprised_blink_left.svg   01                          mini_eyes/surprised/open &
#~ bash layers2pngs.bash  svg/surprised_blink_left.svg   02,01                       mini_eyes/surprised/begin &
#~ bash layers2pngs.bash  svg/surprised_blink_left.svg   01,03,06,08,10,08,06,03,01  mini_eyes/surprised/blink

