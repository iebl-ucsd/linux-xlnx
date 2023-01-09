#!/bin/bash
found=false;
for i in `ls openwrt_patches/pending-5.10`
do
	# if [ "$found" = false ]; then
	# 	if [[ "$i" =~ ^801-v6.1-0001-nvmem-add-driver-hand.* ]]; then
	# 		found=true
	# 	fi
	# 	continue
	# fi
	echo $i
	git am openwrt_patches/pending-5.10/$i || break
done
