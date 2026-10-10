#!/bin/sh

# Fetches the Yale Bright Star Catalog (VizieR V/50) into ./bright-star-catalog.tsv,
# the file scope reads by default (--catalog-path). One star per line:
#   RA(deg)|Dec(deg)|HR|Multiple|Vmag     (J2000, decimal degrees)
#
# The request is LOST's: the VizieR query form as copied from Firefox dev tools.
# The grep keeps the data rows and drops VizieR's header.

url='https://vizier.cds.unistra.fr/viz-bin/asu-tsv'
query='-ref=VIZ5f9b96c42e6d&-to=4&-from=-3&-this=-3&%2F%2Fsource=V%2F50%2Fcatalog&%2F%2Ftables=V%2F50%2Fcatalog&-out.max=unlimited&%2F%2FCDSportal=http%3A%2F%2Fcdsportal.u-strasbg.fr%2FStoreVizierData.html&-out.form=%7C+-Separated-Values&-out.add=_RAJ%2C_DEJ&%2F%2Foutaddvalue=default&-oc.form=dec&-nav=cat%3AV%2F50%26tab%3A%7BV%2F50%2Fcatalog%7D%26key%3Asource%3DV%2F50%2Fcatalog%26HTTPPRM%3A%26&-c=&-c.eq=J2000&-c.r=++2&-c.u=arcmin&-c.geom=r&-source=V%2F50%2Fcatalog&-order=I&recno=&-out=HR&HR=&Name=&DM=&HD=&SAO=&FK5=&IRflag=&r_IRflag=&-out=Multiple&Multiple=&ADS=&ADScomp=&VarID=&RAJ2000=&DEJ2000=&GLON=&GLAT=&-out=Vmag&Vmag=&n_Vmag=&u_Vmag=&B-V=&u_B-V=&U-B=&u_U-B=&R-I=&n_R-I=&SpType=&n_SpType=&pmRA=&pmDE=&n_Parallax=&Parallax=&RadVel=&n_RadVel=&l_RotVel=&RotVel=&u_RotVel=&Dmag=&Sep=&MultID=&MultCnt=&NoteFlag=&%2F%2Fnoneucd1p=on&-file=.&-meta.ucd=2&-meta=1&-meta.foot=1&-usenav=1&-bmark=POST'
out=bright-star-catalog.tsv

if command -v curl > /dev/null; then
    fetch() {
        curl -sS --compressed -H 'Content-Type: application/x-www-form-urlencoded' \
            -H 'Origin: https://vizier.u-strasbg.fr' --data-raw "$query" "$url"
    }
elif command -v wget > /dev/null; then
    fetch() {
        wget -q -O - --header='Origin: https://vizier.u-strasbg.fr' --post-data="$query" "$url"
    }
else
    echo "download-bsc.sh: needs curl or wget" >&2
    exit 1
fi

# Write to a temporary file first so a failed download never leaves an empty
# catalog behind or replaces a good one.
fetch | grep -E '^[0-9]{3}\.[0-9]{6}' > "$out.tmp"
if [ ! -s "$out.tmp" ]; then
    rm -f "$out.tmp"
    echo "download-bsc.sh: no catalog rows received from $url" >&2
    exit 1
fi
mv "$out.tmp" "$out"
echo "Wrote $(wc -l < "$out" | tr -d ' ') stars to $out"
