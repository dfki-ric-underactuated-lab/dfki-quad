#!/bin/bash

# launch new shell for latest running container
docker exec -it $(docker ps -q -n1) /bin/bash

exit 0
