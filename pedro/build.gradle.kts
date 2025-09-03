plugins {
    kotlin("android")
    id("com.android.library")
}

android {
    namespace = "dev.nextftc.hardware"
    compileSdk = 35

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += "-Xjvm-default=all"
    }

    publishing {
        singleVariant("release")
    }
}

dependencies {
    implementation(libs.bundles.nextftc)
    implementation(libs.pedro)
    compileOnly(libs.bundles.ftc)
}

version = property("versions.pedro") as String
description =
    "NextFTC's extension to add support for Pedro Pathing."

nextFTCPublishing {
    displayName = "NextFTC Extensions - Pedro"
    logoPath = "../assets/logo-icon.svg"
}