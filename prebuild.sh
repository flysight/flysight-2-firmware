#!/bin/bash

GIT_TAG=$(git describe --tags --always)

cat << EOF > ../FlySight/version.h
#ifndef VERSION_H
#define VERSION_H

#define GIT_TAG "${GIT_TAG}"

#endif // VERSION_H
EOF

