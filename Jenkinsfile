properties(
    [
    buildDiscarder
        (logRotator (
            artifactDaysToKeepStr: '',
            artifactNumToKeepStr: '',
            daysToKeepStr: '14',
            numToKeepStr: '10'
        ) ),
    disableConcurrentBuilds()
    ]
)

pipeline {
    agent any

    environment {
        container_name = "c_${BUILD_ID}_${JENKINS_NODE_COOKIE}"
        image_tag = "develop"
        user_ci = credentials('lsst-io')
        work_branches = "${CHANGE_BRANCH} ${GIT_BRANCH} develop"
        // ts_m2com url
        M2COM_URL = "https://github.com/lsst-ts/ts_m2com.git"
        // Authority to publish the document online
        LTD_USERNAME = "${user_ci_USR}"
        LTD_PASSWORD = "${user_ci_PSW}"
        DOCUMENT_NAME = "ts-m2"
    }

    stages {
        stage("Pull docker image") {
            steps {
                script {
                    sh """
                    docker pull lsstts/develop-env:\${image_tag}
                    """
                }
            }
        }
        stage("Prepare Workspace") {
            steps {
                script {
                    sh """
                    chmod -R a+rw \${WORKSPACE} || echo "Failed to set workspace mode"
                    container=\$(docker run -v \${WORKSPACE}:/home/saluser/repo/ -td --rm --name \${container_name} -e LTD_USERNAME=\${user_ci_USR} -e LTD_PASSWORD=\${user_ci_PSW} lsstts/develop-env:\${image_tag})
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd /home/saluser/repos/ts_config_mttcs && /home/saluser/.checkout_repo.sh \${work_branches}\"
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd /home/saluser/repos/ts_xml && /home/saluser/.checkout_repo.sh \${work_branches}\"
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd /home/saluser/repos/ts_idl && /home/saluser/.checkout_repo.sh \${work_branches}\"
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd /home/saluser/repos/ts_sal && /home/saluser/.checkout_repo.sh \${work_branches}\"
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd /home/saluser/repos && git clone ${M2COM_URL} && cd ts_m2com/ && /home/saluser/.checkout_repo.sh \${work_branches}\"
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && source ~/.bashrc && make_idl_files.py MTM2 MTMount\"
                    """
                }
            }
        }
        stage("Running tests") {
            steps {
                script {
                    sh """
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd ~/repos/ts_m2com && eups declare -r . -t saluser && cd ~/repo && eups declare -r . -t saluser && setup ts_m2 -t saluser && pytest -v\"
                    """
                }
            }
        }
        stage("Build and Upload Documentation") {
            steps {
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                    docker exec -u saluser \${container_name} sh -c \"source ~/.setup.sh && cd repo && eups declare -r . -t saluser && setup ts_m2 -t saluser && package-docs build && ltd upload --product ${DOCUMENT_NAME} --git-ref ${GIT_BRANCH} --dir doc/_build/html\"
                    """
                }
            }
        }
    }
    post {
        cleanup {
            sh """
                docker exec -u root --privileged \${container_name} sh -c \"chmod -R a+rw /home/saluser/repos/ts_m2com/ \"
                docker exec -u root --privileged \${container_name} sh -c \"chmod -R a+rw /home/saluser/repo/ \"
                docker stop \${container_name}
            """
        }
    }
}
