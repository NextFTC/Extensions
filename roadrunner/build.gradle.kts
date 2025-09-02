plugins {
    kotlin("android")
    id("com.android.library")
}

android {
    namespace = "dev.nextftc.extensions.roadrunner"
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
    implementation(libs.bundles.roadrunner)
    implementation(libs.bundles.ftc)
}

description =
    "Support for using RoadRunner with NextFTC."

nextFTCPublishing {
    displayName = "NextFTC Extensions - RoadRunner"
    logoPath = "../assets/logo-icon.svg"
    version = property("versions.roadrunner") as String
}