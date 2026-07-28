About scene:

EFORE YOU START:
- you need Unity 6.3 or higher 
- you need HD SRP pipeline 17.3 if you use higher etc custom shaders could not work but seems they should. 
That's why we provide 17.3 version which seems to work with much higher versions aswell. 
For all higher RP versions please use 17.3 HD RP support pack.

Step 1
	- !!!! IMPORTANT !!!! Open "Project settings" ->"Gaphics"-> "Pipline Specific Settings" ->  "Diffusion Profile List"
	and drag and drop our SSS settings diffusion profiles for foliage and water into Diffusion profile list:
		  NM_SSSSettings_Skin_Foliage
		  NM_SSSSettings_Skin_NM Foliage
		  NM_SSSSettings_Skin_NM Foliage Trees
	Without this foliage, materials will not become affected by scattering and they will look wrong.

Step 2 Go to quality settings and quality and set:
	- Set VSync to don't sync

Setp 3 Find HD SRP Demo Small and open it.

Step 4 - HIT PLAY!:)

Play with it, give us feedback and learn about hd srp power.

IMPORTANT:
If you notice in console error:
No more space in Reflection Probe Atlas. To solve this issue, increase the size of the Reflection Probe Atlas in the HDRP settings.
UnityEngine.GUIUtility:ProcessEvent (int,intptr,bool&)
Just change reflection atlas size at hd rp settings into 4kx8k. 