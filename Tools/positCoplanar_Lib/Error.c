#include <stdio.h>
#include <math.h>
#include <malloc.h>

static double x;

#define round(a) (x=(a),(fabs(x - ceil(x))) < (fabs(x - floor(x))) ? \
(ceil(x)) : (floor(x)))

/**********************************************************************************************************/
void Error(NP,impts,obpts,f,Rotat,Translat,Er,Epr,Erhvmax)
/*Error retourne differentes mesures d'erreurs fondees sur les ecarts Image TPP reconstruite/Image de*/
/*depart:*/
/*E est la distance euclidienne moyenne entre points images reconstruits et points images originaux*/
/*Ep est la somme des ecarts horizontaux et verticaux en pixels*/
/*Ehvmax est l'ecart horizontal ou vertical maximum rencontre (valeurs non arrondies en pixels)*/

double  **impts,**obpts,*Er,*Erhvmax;
double  f;
double  Rotat[3][3],Translat[3];
long int    NP;
long int    *Epr;

{
void   PerspMoveAndProj();
double  **impredic,**ErVect;
long int    i,j,fr;

/*allocations*/
impredic=(double **)malloc(NP * sizeof(double *));
ErVect=(double **)malloc(NP * sizeof(double *));
for (i=0;i<NP;i++)
  {
    impredic[i]=(double *)malloc(2 * sizeof(double));
    ErVect[i]=(double *)malloc(2 * sizeof(double));
  }

if ((Rotat[0][0])!=2.0) /*un "2" en premiere position des matrices de rotation signifie que la pose est*/
                        /*impossible (points derriere le plan image, par exemple)*/
  {
    PerspMoveAndProj(NP,obpts,Rotat,Translat,f,impredic); /*calcul de l'image TPP reconstruite pour la*/
                                                          /*pose consideree*/
    for (i=0;i<NP;i++)
      {
	for (j=0;j<2;j++) ErVect[i][j]=impredic[i][j]-impts[i][j];
      }

    /*mesures des erreurs*/
    *Er=0.0;
    *Epr=0;
    *Erhvmax=0;
    for (i=0;i<NP;i++)
      {
	*Er+=sqrt(ErVect[i][0]*ErVect[i][0]+ErVect[i][1]*ErVect[i][1]);
	*Epr+=(long int)(fabs(round(impredic[i][0])-round(impts[i][0]))+
		fabs(round(impredic[i][1])-round(impts[i][1])));
	if (fabs(ErVect[i][0])>*Erhvmax) 
	  *Erhvmax=fabs(ErVect[i][0]);
	if (fabs(ErVect[i][1])>*Erhvmax) 
	  *Erhvmax=fabs(ErVect[i][1]);
      }
    *Er=*Er/NP;
  }
else /*erreurs=-1 en cas de pose impossible*/
  {
    *Er=-1.0;
    *Epr=-1.0;
    *Erhvmax=-1.0;
  }

/*desallocations*/
for (fr=0;fr<NP;fr++)
  {
    free(impredic[fr]);
    free(ErVect[fr]);
  }
}


/**********************************************************************************************************/
void  PerspMoveAndProj(N,obj,r,t,foc,proj) /*synthese d'une image TPP pour la pose consideree (r,t)*/

long int    N;
double  **obj,**proj;
double  r[3][3],t[3];
double  foc;

{
double  **moved;
long int    i,j,k;


/*allocations*/
moved=(double **)malloc(N * sizeof(double *));
for (i=0;i<N;i++) moved[i]=(double *)malloc(3 * sizeof(double));

for (i=0;i<N;i++)
  {
    for (j=0;j<3;j++) moved[i][j]=t[j];
  }
for (i=0;i<N;i++)
  {
    for (j=0;j<3;j++)
      {
	for (k=0;k<3;k++) moved[i][j]+=r[j][k]*obj[i][k];
      }
  }
for (i=0;i<N;i++)
  {
    for (j=0;j<2;j++)	proj[i][j]=foc*moved[i][j]/moved[i][2];
  }

/*desallocations*/
for (i=0;i<N;i++) free(moved[i]);



}



























