/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import frc.robot.Constants
import frc.robot.interfaces.LimelightService
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.net.URL
import java.net.http.HttpClient
import java.net.http.HttpRequest
import java.net.http.HttpResponse
import java.time.Duration
//for multiple limelights make it a class
object HttpLimelightService : LimelightService {
    private val client =
        HttpClient
            .newBuilder()
            .version(HttpClient.Version.HTTP_1_1)
            .connectTimeout(
                Duration.ofMillis(
                    Constants.LimelightConstants.TIMEOUT,
                ),
            ).build()

    @Suppress("TooGenericExceptionCaught")
    override suspend fun fetchResults(): String? {
        var result: String?
        try {
            val response =
                withContext(Dispatchers.IO) {
                    client.send(
                        HttpRequest
                            .newBuilder()
                            .uri(
                                URL(
                                    "http://${Constants.LimelightConstants.IP_ADDR}:5807/results",
                                ).toURI(),
                            ).timeout(
                                Duration.ofMillis(
                                    Constants.LimelightConstants.TIMEOUT,
                                ),
                            ).GET()
                            .build(),
                        HttpResponse.BodyHandlers.ofString(),
                    )
                }
            result = response.body()
        } catch (e: Exception) {
            println(e)
            println("shit")
            result = null
        }
        return result
    }
}
