#include <fstream>
#include <sstream>
#include <stdexcept>
#include <stdlib.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/tokenizer.hpp>

void getKineoPathFromFile (const char* fileName, CkwsPathShPtr& path)
{
  ifstream file;
  file.open (fileName);

  if(file.is_open())
    cout << "File " << fileName << " successfully opened.\n";
  else
    cout << "Error opening " << fileName << ".\n";

  std::vector <CkwsConfigShPtr> configVector;
 
  while (!file.eof ())
    {
      std::string line;
      std::getline (file, line);
      std::string configStartMarkup = "<CONFIG_VALUES>";
      std::string configEndMarkup = "</CONFIG_VALUES>";
      std::size_t configStart = line.find (configStartMarkup);
      std::size_t configEnd = line.find (configEndMarkup);

      if (configStart != std::string::npos
	  && configEnd != std::string::npos)
	{
	  configStart += configStartMarkup.length ();
	  --configEnd;
	  std::string configValues
	    = line.substr (configStart,
			   line.length ()
			   - configStart
			   - configEndMarkup.length () - 1);

	  boost::char_delimiters_separator<char>
	    separator (false, "", " ");
	  boost::tokenizer<> tokenizer (configValues, separator);

	  unsigned dofId = 0;
	  CkwsConfigShPtr configuration
	    = CkwsConfig::create (path->device ());
	  for (boost::tokenizer<>::iterator it=tokenizer.begin();
	       it!=tokenizer.end();
	       ++it)
	    {
	      double dofValue = std::atof ((*it).c_str ());
	      configuration->dofValue (dofId, dofValue);
	      ++dofId;
	    }
	  
	  assert (dofId == path->device ()->countDofs ());
	  configVector.push_back (configuration);
	}
    }

  file.close ();

  for (unsigned i = 0; i < configVector.size () - 1; ++i)
    if (KD_OK !=
	path->appendDirectPath (configVector[i], configVector[i + 1]))
      {
	std::cerr << "Could not append direct path " << i << std::endl;
      }
  
  std::cout << "number of direct paths: "
	    << path->countDirectPaths () << std::endl;  
}
