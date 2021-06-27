#include <stdlib.h>
#include <stdio.h>
#include <string.h>




int main (int argc, char **argv)
{
#define MPU_MAXLINE 1024 
	char *keyword = (char *)calloc(MPU_MAXLINE,sizeof(char));
	char *filen = (char *)calloc(MPU_MAXLINE,sizeof(char));
	FILE *filep;
	do {
		printf("Type a filename:");
		fscanf(stdin, "%s", filen);

		if ( (filep = fopen(filen, "r")) == NULL) {
			printf("File not found, try another name!\n");
		}

	} while (NULL == filep);

	char *buf = (char *)calloc(MPU_MAXLINE, sizeof(char));
	long loffset = 0L;
	int  lineno = 0;

	printf("Type a keyword:");
	fscanf(stdin, "%s", keyword);
	while ( !((feof(filep)) || (ferror(filep))) ) {
		lineno++;
		fscanf(filep, "%s", buf);
		printf("LINE:%lu:%s\n",lineno, buf);
	}
	printf("reached end of file\n");
	rewind(filep);

	/* search first occurrence of keyword in file */
	do { 	

		loffset = ftell(filep);
		if(-1L == loffset) {
			fprintf(stderr, "Unable to obtain offset in %s\n", filen);
			exit(EXIT_FAILURE);
		} else {
			lineno++;
		}

		if ( !fgets(buf, MPU_MAXLINE, filep)){
		       	break;
		}

	} while (strstr(buf, keyword) == NULL);

	if (feof(filep) || ferror(filep)) {
		printf("not found!\n");
		rewind(filep);
	} else {
		printf("%s (%d): %s\n", keyword, lineno, buf);
		fseek(filep, loffset, 0);
	};

	/* find all the occurrences of keyword in file */
	int count = 0;
	printf("Now, all the occurences of keyword\n");
	rewind(filep);
	while (fgets(buf, MPU_MAXLINE, filep) != NULL) {
		if (strstr(buf, keyword) != NULL) {
			lineno++;
			count++;
			printf("%s : %d : line:(%d): %s", keyword, count, lineno, buf);
		}
	}
	printf("Reached end of file\n");
	fclose(filep);

#undef MPU_MAXLINE
}
