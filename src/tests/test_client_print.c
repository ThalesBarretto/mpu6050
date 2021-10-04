

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>

#define BUF_SIZE 500	/* max size of message in bytes */

int main(int argc, char *argv[])
{
	struct addrinfo		hints;		/* search criteria		*/
	struct addrinfo		*result;	/* list of struct addrin	*/
	struct addrinfo		*rp;		/* list iterator		*/
	int 			sfd;		/* socket file descriptor	*/
	int 			s;		/* return val. from getaddrinfo	*/
	size_t			len;		/* length of each message	*/
	ssize_t			nread;		/* bytes read			*/
	char			buf[BUF_SIZE];	/* stores the message		*/

	/* Check syntax */
        if (argc < 3) {
		printf("usage: %s <host> <port> <message> [messages...]\n", argv[0]);
		exit(EXIT_FAILURE);
        }

        /* Prepare hints: datagram, any protocol, any address */
	memset(&hints, 0, sizeof(hints));
	hints.ai_family		= AF_INET;	/* Allow IPv4 or IPv6		*/
	hints.ai_socktype	= SOCK_DGRAM;	/* For datagram socket		*/ 
	hints.ai_protocol	= 0;		/* For any protocol		*/
	hints.ai_flags		= 0;		/* no special flags		*/
	hints.ai_canonname	= NULL;		/* on hints must be NULL	*/
	hints.ai_addr		= NULL;		/* on hints must be NULL	*/
	hints.ai_next		= NULL;		/* on hints must be NULL	*/

	/* Build an addrinfo list matching the desired configuration */
        s = getaddrinfo(argv[1], argv[2], &hints, &result);
        if (s != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
		exit(EXIT_FAILURE);
        }

        /* Have list. Try socket(), connect(), proceed on success. */
	for (rp = result; rp != NULL; rp = rp->ai_next) {
		sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (sfd == -1)
			continue;	/* socket() failed, try next */

		if (connect(sfd, rp->ai_addr, rp->ai_addrlen) != -1)
			break;		/* connect() success, proceed */

	       close(sfd);		/* connect() failed, try next */
        }

	freeaddrinfo(result);	/* discard the list */

	if (rp == NULL) {	/* failed the whole list  */
		fprintf(stderr, "Could not connect\n");
		exit(EXIT_FAILURE);
	}

	/* Have channel. Send separate datagrams, read responses. */ 
	for (int j = 3; j < argc; j++) {

		len = strlen(argv[j]) + 1;		/* mind the terminator */

		if (len > BUF_SIZE) {			/* buffer overflow */
			fprintf(stderr, "Ignoring long message in argument %d\n", j);
			continue;
		}

		if (write(sfd, argv[j], len) != len) {	/* write failed */
			fprintf(stderr,"partial/failed write\n");
			exit(EXIT_FAILURE);
		}

	}
	/* don't forget to close */
	close(sfd);
	exit(EXIT_SUCCESS);
}

