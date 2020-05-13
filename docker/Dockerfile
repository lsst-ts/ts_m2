ARG deploy_tag=salobj_v5.7.0

FROM lsstts/deploy-env:${deploy_tag}

LABEL maintainer Tiago Ribeiro <tribeiro@lsst.org>

ARG m2=0.1.dev9+geb50cc5
ARG idl=1.1.3_4.8.0
ARG config=v0.2.0

RUN source /home/saluser/.setup.sh && \
    conda install -c lsstts ts-m2=${m2} ts-idl=${idl}

WORKDIR ${WORKDIR}

RUN git clone https://github.com/lsst-ts/ts_config_mttcs.git

WORKDIR ${WORKDIR}/ts_config_mttcs

RUN git checkout ${config} -b ${config}

WORKDIR /home/saluser/

ENV TS_CONFIG_MTTCS_DIR=${WORKDIR}/ts_config_mttcs

COPY m2_setup.sh /home/saluser/.setup.sh
