#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define BUFLEN 512
#define NPACK 10
#define PORT ((uint16_t)10900)

// Note that this is the "local broadcast" address.  Others exist for 
// wider-ranging use (e.g., across a subnet): check out 
// https://secure.wikimedia.org/wikipedia/en/wiki/Broadcast_address for some 
// additional details
#define BROADCAST_ADDRESS   "255.255.255.255"

void diep(const char *s);

void diep(const char *s)
{
  perror(s);
  exit(1);
}

void help(const char * const programName, FILE* out) {
  fprintf(out, "Usage: %s <cmdToSend>\n", programName);
  // fprintf(out, "Where <cmdToSend> is 'on' or 'off'\n");
}

int main(int argc, const char * argv[])
{
  if ( argc != 2 ) {
    help(argv[0], stderr);
    return -1;
  }

  const char* const commandToSend = argv[1];

  struct sockaddr_in si_other;
  int s;
  socklen_t slen=sizeof(si_other);

  if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    diep("socket");
    
  // If we're trying to use a broadcast address, we will need to set the 
  // correct option on the socket first.
  int broadcast = 1;      /* Note: on some platforms, this may need to be "char" */
  if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast,
    sizeof broadcast) == -1) {
    diep("setsockopt (SO_BROADCAST)");
  }

  memset((char *) &si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(PORT);
  if (inet_aton(BROADCAST_ADDRESS, &si_other.sin_addr)==0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

  printf("Sending packet\n");
  if (sendto(s, commandToSend, strlen(commandToSend)+1, 0, (struct sockaddr*)&si_other, slen)==-1) {
    diep("sendto()");
  }

  close(s);
  return 0;
}


