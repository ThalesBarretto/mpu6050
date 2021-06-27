
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
	struct sockaddr_storage	peer_addr;	/* stores sender's addr		*/
	socklen_t		peer_addr_len;	/* length of peer_addr		*/

	/* Check syntax */
        if (argc < 2) {
 	       printf("usage: %s <port>\n", argv[0]);
 	       exit(EXIT_FAILURE);
        }

        /*
	 * Our server will listen, so we need the AI_PASSIVE flag.
	 */
        memset(&hints, 0, sizeof(hints));
        hints.ai_family		= AF_INET;	/* Allow IPv4 or IPv6		*/
        hints.ai_socktype	= SOCK_DGRAM;	/* For datagram socket		*/ 
        hints.ai_protocol	= 0;		/* For default protocol		*/
	hints.ai_flags		= AI_PASSIVE;	/* For wildcard IP address	*/
	hints.ai_canonname	= NULL;		/* on hints must be NULL	*/
	hints.ai_addr		= NULL;		/* on hints must be NULL	*/
	hints.ai_next		= NULL;		/* on hints must be NULL	*/

	/* 
	 * To receive datagrams from any address, pass:
	 * 	host = NULL
	 *
	 * Combined with above hints, the result will have "wildcard address":
	 * 	rp->ai_ddr = (INADDR_ANY for IPv4 | INADDR_ANY_INIT for IPv6)
	 */ 
        s = getaddrinfo(NULL, argv[1], &hints, &result);
        if (s != 0) {
 	       fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
 	       exit(EXIT_FAILURE);
        }	

        /* Create a socket and bind it. */
        for (rp = result; rp != NULL; rp = rp->ai_next) {
 		sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
 		if (sfd == -1)
 			continue;	/* socket() failed, try next */

 		if (bind(sfd, rp->ai_addr, rp->ai_addrlen) != -1)
 		       break;		/* bind() success, proceed */

 	       close(sfd);		/* bind() failed, try next */
        }
	freeaddrinfo(result);	/* discard the list */

	if (rp == NULL) {	/* failed the whole list  */
 	       fprintf(stderr, "Could not bind\n");
 	       exit(EXIT_FAILURE);
        }

	/* 
	 * We use recvfrom() to retrieve the sender addr inside peer_addr.
	 *
	 * We use getnameinfo() to retrives sender's hostname and port inside
	 * host and serv. The flag NI_NUMERICSERV gets the port number instead of
	 * the service name. We could also use the NI_NUMERICHOST to get the 
	 * numeric address of the host instead of the hostname.
	 */
        for (;;) {
		peer_addr_len = sizeof(peer_addr);
		nread = 0;
		nread = recvfrom(sfd, buf, BUF_SIZE, 0,
				(struct sockaddr *) &peer_addr, &peer_addr_len);

		if (nread == -1)
			continue;	/* ignore failed read request */

		char host[NI_MAXHOST];		/* sender's hostname	*/
		char service[NI_MAXSERV];	/* sender's portnumber	*/

		s = getnameinfo((struct sockaddr *)&peer_addr,
			       	peer_addr_len,
				host, NI_MAXHOST,
			        service, NI_MAXSERV,
				NI_NUMERICSERV);
//		if (s == 0)
//			printf("Received %d bytes from %s:%s\n", nread, host, service);
//		else
//			fprintf(stderr, "getnameinfo: %s\n", gai_strerror(s));

	 	/* print received message */
		printf("%s",buf);
	}

	/* don't forget to close */
	close(sfd);

	exit(EXIT_SUCCESS);
}
