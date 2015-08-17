#include <stdio.h>
#include <malloc.h>
#include <math.h>

/*********************************************************************************************************/
void Composit(np,coplImage,copl,fLength,POSITRot1,POSITTrans1,POSITRot2,POSITTrans2)
/*Retourne le nombre de poses DIFFERENTES acceptables (cf plus bas pour cette notion) en nsol, ainsi que*/ 
/*la meilleure pose (rota1, transa1), i.e. la plus proche en rotation (transmission de rota) lorsque 2*/
/*sont acceptables, ou celle donnant la plus faible erreur E s'il n'y en a pas 2 acceptables.*/

long int   np;
double **coplImage,**copl;
double  POSITRot1[3][3],POSITTrans1[3],POSITRot2[3][3],POSITTrans2[3];
double fLength;

{
long int i,j,Ep1,Ep2,fr;
double   E1,E2,Ehvmax1,Ehvmax2,noise_quantif=0.5,rotEr1,rotEr2,rotEc;
double   **coplVectors,**coplMatrix;


void     PositCopl(); /*cf file PositCopl.c*/
void	 PseudoInverse(); /*cf file svd.c*/

/*allocations*/
coplVectors=(double **)malloc(np * sizeof(double *));
coplMatrix=(double **)malloc(3 * sizeof(double *));
for (i=0;i<np;i++) coplVectors[i]=(double *)malloc(3 * sizeof(double));
for (i=0;i<3;i++) coplMatrix[i]=(double *)malloc(np * sizeof(double));

for (i=0;i<np;i++)
  {
    coplVectors[i][0]=copl[i][0]-copl[0][0];
    coplVectors[i][1]=copl[i][1]-copl[0][1];
    coplVectors[i][2]=copl[i][2]-copl[0][2];
  }

PseudoInverse(coplVectors,np,coplMatrix); /*coplMatrix est la pseudoinverse de coplVectors*/

/*for (i=0;i<3;i++)*/
/*{*/
/*  printf("\n");*/
/*  for (j=0;j<np;j++) printf("%e ",coplMatrix[i][j]);*/
/*}*/

PositCopl(np,coplImage,copl,coplMatrix,fLength,
          POSITRot1,POSITTrans1,POSITRot2,POSITTrans2);
/*retourne les DEUX poses resultant de la convergence des deux branches de POSIT, sans les juger*/

/*calcul des translations d'origine a origine  (l'origine du rep. objet n'est pas forcement*/
/*confondue avec Mo)*/
for (i=0;i<3;i++)
  {
    for (j=0;j<3;j++)
	{
	  POSITTrans1[i]-=POSITRot1[i][j]*copl[0][j];
	  POSITTrans2[i]-=POSITRot2[i][j]*copl[0][j];
	}
  }

/* if ((POSITRot1[0][0]==2)&&(POSITRot2[0][0]==2))...CAS A IMPLEMENTER (n'apparait jamais)*/

/*desallocations*/
for (fr=0;fr<np;fr++) free(coplVectors[fr]);
for (fr=0;fr<3;fr++) free(coplMatrix[fr]);

}







