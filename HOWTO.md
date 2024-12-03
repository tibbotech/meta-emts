# How To 

...start your own SP7021-based SOC software project and get the ISPBOOOT.BIN
image that fits your board peripherial and software requirements.

## Steps:

1. Clone this repo

2. Rename the folder name to "meta-[mycompany]-[prj0]". 
Search for string "example0" inside and fix the names accordinly.

3. Create your own DTS for your board, place it to 
recipes-kernel/linux/linux-sp-5.4/dts/
as the patch, give it the name and fix the file name at
linux-sp_5.4.bbappend file.

4. Check the contents of the image recipe at recipes-core/images/,
include the recipe names you need to have in your resulting image.

5. Fix the README.md

6. Push the result into your Public or Private git[Hub]
(you may ask Tibbo to keep your project).



