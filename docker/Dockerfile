ARG image_tag=develop
FROM lsstts/salobj:${image_tag}

ARG m2_branch=develop
ARG config_mttcs_branch=develop

WORKDIR /home/saluser/repos/

RUN git clone https://github.com/lsst-ts/ts_config_mttcs.git && \
    git clone https://github.com/lsst-ts/ts_m2.git

WORKDIR /home/saluser/repos/ts_config_mttcs

RUN source /opt/lsst/software/stack/loadLSST.bash && \
    source /home/saluser/repos/ts_sal/setup.env && \
    git checkout ${config_mttcs_branch} && \
    eups declare -r . -t current

WORKDIR /home/saluser/ts_m2

RUN source /opt/lsst/software/stack/loadLSST.bash && \
    source /home/saluser/repos/ts_sal/setup.env && \
    git checkout ${m2_branch} && \
    git pull && \
    eups declare -r . -t current && \
    setup ts_m2 -t current && \
    make_idl_files.py MTM2 &> /dev/null && \
    scons

WORKDIR /home/saluser/

COPY m2_setup.sh /home/saluser/.setup.sh

