#find ./ -iname "*" -type f -exec sh -c 'echo "${0}"' {} \;
find ./ -iname "*" -type f -exec sh -c 'm2r "${0}"' {} \;

