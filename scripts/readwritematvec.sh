#!/bin/sh

if [ $# = 2 ]
then 
    echo "Correct call!"
    OUT=$2
elif [ $# = 1 ]
then
    echo "Acceptable call!"
    a="$1"
    OUT="${a%.*}.mat"
else
    echo "Wrong call - quitting!"
fi
echo "Preprocessing mtx files"
awkcmd1='BEGIN{mstart=0;}
  ($1~/^\*M/){mstart++; next}
  (mstart==1){if($1!~/^\*/){print}else{exit}}'
awkcmd2='BEGIN{RS=",";ORS="\n"}{print}'
gawk "$awkcmd1" $1|gawk "$awkcmd2"|gawk '(NF!=0){print}' > ./.STIFFNESS.mtx

awkcmd1='BEGIN{mstart=0;}
  ($1~/^\*M/){mstart++; next}
  (mstart==2){if($1!~/^\*/){print}else{exit}}'
awkcmd2='BEGIN{RS=",";ORS="\n"}{print}'
gawk "$awkcmd1" $1|gawk "$awkcmd2"|gawk '(NF!=0){print}' > ./.MASS.mtx

awkcmd1='BEGIN{vstart=0;}
  ($0~/^\*\*\*C/){vstart++; next}
  (vstart>0){print $2,$3,$4}
  (vstart>0 && $1!~/^\*\*/){exit}'
awkcmd2='BEGIN{FS=","}
  {for(i=1;i<=NF;i++)
    printf("%s ", $i);
    printf("\n")}'
gawk "$awkcmd1" $1|gawk "$awkcmd2" > ./.FVEC.mtx

awkcmd1='BEGIN{rstart=0}
 ($0~/^\*\* SUBSTRUCTURE REC/){rstart++;
 if(rstart>1){printf("\n")}
 next}
 (rstart!=0 && $1~/\*\*/){
 for(i=2;i<=NF;i++)printf("%s",$i);}'
awkcmd2='BEGIN{FS=","}
  {for(i=1;i<=NF;i++)
    printf("%s ",$i);
    printf("\n");}'
gawk "$awkcmd1" $1|gawk "$awkcmd2" > .RECOV.mtx

echo "Preprocessing mtx files done"

python <<EOF
import numpy as np
import scipy.io as io

print("Reading Mass Matrix from mtx file.");
Mv = np.loadtxt('.MASS.mtx');
print("Done.");

print("Reading Stiffness Matrix from mtx file.");
Kv = np.loadtxt('.STIFFNESS.mtx');
print("Done.");

print("Reading Recovery Matrix from mtx file.");
R = np.loadtxt('.RECOV.mtx');
print("Done.");

print("Processing Matrices.")

Nelm = len(Mv);
Nelk = len(Kv);
if (Nelm!=Nelk):
        sys.exit("GIGO - Mass & Stiffness not of same length.");
Nel = Nelm;

Nd = ((np.sqrt(1+8*Nel)-1)/2).astype(int); # Solution of Nd(Nd+1)/2-Nel = 0

M = np.zeros((Nd,Nd));
K = np.zeros((Nd,Nd));

(xi,yi) = np.tril_indices(Nd);
M[xi,yi] = Mv;
M[yi,xi] = Mv;
K[xi,yi] = Kv;
K[yi,xi] = Kv;

print("Done.")

print("Reading Forcing Vector from mtx file.");
Fvdat = np.loadtxt('.FVEC.mtx');
print("Done.");

print("Processing Force Vector.");
Fv = np.zeros(M.shape[0]);
ids = range(np.where(np.diff(Fvdat[:, 1])==0)[0][0], Fvdat.shape[0])
n1dofnds = Fvdat[ids, 0].astype(int)
n3dofnds = Fvdat[list(set(range(Fvdat.shape[0]))-set(ids)), 0].astype(int)
Fv[((n3dofnds-1)*3+np.kron(np.ones(int(len(n3dofnds)/3)),[0, 1, 2])).astype(int)] = \
Fvdat[list(set(range(Fvdat.shape[0]))-set(ids)), 2]
print("Whew.")
Fv[range(-len(n1dofnds), 0)] = Fvdat[ids, 2]
print("Done.")

print("Matrix extraction complete - writing mat file")
dict = {"M": M, "K": K, "R": R.T, "Fv": Fv.reshape((len(Fv),1))};
io.savemat(".out.mat",dict);
print("Processing Over")
EOF
mv .out.mat $OUT
# rm .STIFFNESS.mtx .MASS.mtx .RECOV.mtx .FVEC.mtx
