[CmdletBinding()]
param([double]$Seconds=120, [string]$Url='ws://127.0.0.1:8765', [string]$Output='reports/foxglove_load.json')
$ErrorActionPreference='Stop'
$client=[System.Net.WebSockets.ClientWebSocket]::new()
$client.Options.AddSubProtocol('foxglove.sdk.v1')
$cancel=[System.Threading.CancellationTokenSource]::new()
$cancel.CancelAfter([TimeSpan]::FromSeconds($Seconds+10))
$buffer=New-Object byte[] 1048576
$channels=@{}; $counts=@{}; $messages=0; $bytes=0
try {
    $null=$client.ConnectAsync([Uri]$Url,$cancel.Token).GetAwaiter().GetResult()
    $clock=[Diagnostics.Stopwatch]::StartNew()
    while ($clock.Elapsed.TotalSeconds -lt $Seconds) {
        $stream=[IO.MemoryStream]::new()
        do {
            $result=$client.ReceiveAsync([ArraySegment[byte]]::new($buffer),$cancel.Token).GetAwaiter().GetResult()
            $stream.Write($buffer,0,$result.Count)
        } until ($result.EndOfMessage)
        $data=$stream.ToArray();$stream.Dispose()
        if ($result.MessageType -eq [System.Net.WebSockets.WebSocketMessageType]::Close) { break }
        if ($result.MessageType -eq [System.Net.WebSockets.WebSocketMessageType]::Text) {
            $message=[Text.Encoding]::UTF8.GetString($data)|ConvertFrom-Json
            if ($message.op -eq 'advertise') {
                $subscriptions=@()
                foreach ($channel in $message.channels) {
                    if ($channel.topic -match '/preview/|/semantic_preview/|^/tf|^/clock$|/state/odometry$|/perception/markers$') {
                        $channels[[int]$channel.id]=$channel.topic
                        $subscriptions+=@{id=[int]$channel.id;channelId=[int]$channel.id}
                    }
                }
                if ($subscriptions.Count) {
                    $payload=[Text.Encoding]::UTF8.GetBytes((@{op='subscribe';subscriptions=$subscriptions}|ConvertTo-Json -Depth 5 -Compress))
                    $null=$client.SendAsync([ArraySegment[byte]]::new($payload),[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$cancel.Token).GetAwaiter().GetResult()
                }
            }
        } elseif ($data.Length -ge 13 -and $data[0] -eq 1) {
            $id=[BitConverter]::ToUInt32($data,1);$topic=$channels[[int]$id]
            if ($topic) { $counts[$topic]=1+$counts[$topic] }
            $messages++;$bytes+=$data.Length
        }
    }
    @{duration_s=$clock.Elapsed.TotalSeconds;messages=$messages;bytes=$bytes;topics=$counts}|ConvertTo-Json -Depth 5|Set-Content -LiteralPath $Output -Encoding UTF8
} finally {
    $client.Dispose();$cancel.Dispose()
}
