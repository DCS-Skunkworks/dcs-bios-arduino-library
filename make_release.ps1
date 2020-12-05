$version = Read-Host -Prompt 'What version are you release (i.e. 0.3.1)'
$outFile = './Releases/dcs-bios-arduino-library-' + $version + '.zip'

if (!(Test-Path './Releases'))
{
    new-item './Releases' -itemtype directory
}

Get-ChildItem -Path '.\' -Exclude 'Releases','make_release.ps1' | Compress-Archive -DestinationPath $outfile