plugins {
    kotlin("jvm") version libs.versions.kotlin.get()
    `java-library`
    alias(libs.plugins.deployer)
    alias(libs.plugins.dokka)
}

group = "dev.nextftc"
version = property("version") as String

dependencies {
    testImplementation(libs.bundles.kotest)
    testImplementation(libs.mockk) // if needed
    api(libs.functional.interfaces) // if needed
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(17)
}

val dokkaJar = tasks.register<Jar>("dokkaJar") {
    dependsOn(tasks.named("dokkaGenerate"))
    from(dokka.basePublicationsDirectory.dir("html"))
    archiveClassifier = "html-docs"
}

deployer {
    projectInfo {
        name = "Extensions"
        description = "Extensions to integrate NextFTC with other libraries"
        url = "https://nextftc.dev/extensions"
        scm {
            fromGithub("NextFTC", "Extensions")
        }
        license("GNU General Public License, version 3", "https://www.gnu.org/licenses/gpl-3.0.html")

        // TODO: for each developer:
        developer("Rowan McAlpin", "rowan@nextftc.dev", url = "https://rowanmcalpin.com")
    }

    signing {
        key = secret("MVN_GPG_KEY")
        password = secret("MVN_GPG_PASSWORD")
    }

    content {
        kotlinComponents {
            kotlinSources()
            docs(dokkaJar)
        }
    }

    localSpec {
        release.version = "$version-LOCAL"
    }

    nexusSpec("snapshot") {
        release.version = "$version-SNAPSHOT"
        repositoryUrl = "https://central.sonatype.com/repository/maven-snapshots/"
        auth {
            user = secret("SONATYPE_USERNAME")
            password = secret("SONATYPE_PASSWORD")
        }
    }

    centralPortalSpec {
        auth {
            user = secret("SONATYPE_USERNAME")
            password = secret("SONATYPE_PASSWORD")
        }
        allowMavenCentralSync = (property("automaticMavenCentralSync") as String).toBoolean()
    }
}

//TODO: Configure Dokka if desired