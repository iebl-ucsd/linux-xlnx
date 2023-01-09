#!/bin/bash
found=false;
for i in `ls openwrt_patches/pending-5.10`
do
	if [ "$found" = false ]; then
		if [[ "$i" =~ ^101-Use-stddefs.h.* ]]; then
			found=true
		fi
		continue
	fi
	echo $i
	git am openwrt_patches/pending-5.10/$i || break
done
