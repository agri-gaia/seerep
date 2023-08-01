#include "seerep_core/core.h"

namespace seerep_core
{
Core::Core(std::string dataFolder, bool loadHdf5Files) : m_dataFolder(dataFolder)
{
  if (loadHdf5Files)
  {
    loadProjectsInFolder();
  }
}

Core::~Core()
{
}

seerep_core_msgs::QueryResult Core::getDataset(seerep_core_msgs::Query& query)
{
  if (query.sparqlQuery)
  {
    if (query.label)
    {
      throw std::invalid_argument("label and sparqlQuery are both set in the query. Only use one at a time!");
    }
    else if (!query.ontologyURI)
    {
      throw std::invalid_argument("The ontology URI is not set but it is needed to perform a sparqlQuery!");
    }
    else
    {
      getConceptsViaSparqlQuery(query.sparqlQuery.value(), query.ontologyURI.value(), query.label);
    }
  }

  if (query.label && query.ontologyURI)
  {
    checkForOntologyConcepts(query);
  }

  // search all projects
  if (!query.projects)
  {
    return getDatasetFromAllProjects(query);
  }
  // Search only in project specified in query
  else
  {
    return getDatasetFromSpecificProjects(query);
  }
}

seerep_core_msgs::QueryResult Core::getInstances(const seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;

  // search all projects
  if (!query.projects)
  {
    for (auto& it : m_projects)
    {
      auto instances = it.second->getInstances(query);
      if (!instances.dataOrInstanceUuids.empty())
      {
        result.queryResultProjects.push_back(instances);
      }
    }
  }
  // Search only in project specified in query
  else
  {
    for (auto projectuuid : query.projects.value())
    {
      auto project = findProject(projectuuid);

      auto instances = project->second->getInstances(query);
      if (!instances.dataOrInstanceUuids.empty())
      {
        result.queryResultProjects.push_back(instances);
      }
    }
  }

  return result;
}

std::vector<seerep_core_msgs::ProjectInfo> Core::loadProjectsInFolder()
{
  std::vector<seerep_core_msgs::ProjectInfo> projectInfos;

  for (const auto& entry : std::filesystem::directory_iterator(m_dataFolder))
  {
    if (entry.is_regular_file() && entry.path().filename().extension() == ".h5")
    {
      try
      {
        boost::uuids::string_generator gen;
        boost::uuids::uuid projectUUID = gen(entry.path().stem().string());

        if (m_projects.find(projectUUID) == m_projects.end())
        {
          auto project = std::make_shared<CoreProject>(projectUUID, entry.path().string());
          m_projects.insert(std::make_pair(projectUUID, project));

          // add project to the list of newly indexed projects
          seerep_core_msgs::ProjectInfo projectinfo{ project->getName(), projectUUID, project->getFrameId(),
                                                     project->getGeodeticCoordinates(), project->getVersion() };

          projectInfos.push_back(projectinfo);
        }
      }
      catch (const std::runtime_error& e)
      {
        // just log the error in the server, nothing we can do otherwise
        BOOST_LOG_SEV(m_logger, boost::log::trivial::error)
            << e.what() << "While trying to load new project into the index";
      }
    }
  }
  return projectInfos;
}

void Core::deleteProject(boost::uuids::uuid uuid)
{
  auto project = m_projects.find(uuid);

  if (project != m_projects.end())
  {
    m_projects.erase(project);

    std::string filename = boost::lexical_cast<std::string>(uuid);
    std::string path = m_dataFolder + "/" + filename + ".h5";

    std::filesystem::remove(path);
  }
};

void Core::createProject(const seerep_core_msgs::ProjectInfo& projectInfo)
{
  std::string filename = boost::lexical_cast<std::string>(projectInfo.uuid);
  std::string path = m_dataFolder + "/" + filename + ".h5";

  auto project = std::make_shared<CoreProject>(projectInfo.uuid, path, projectInfo.name, projectInfo.frameId,
                                               projectInfo.geodetCoords, projectInfo.version);
  m_projects.insert(std::make_pair(projectInfo.uuid, project));
}

std::vector<seerep_core_msgs::ProjectInfo> Core::getProjects()
{
  std::vector<seerep_core_msgs::ProjectInfo> projectInfos;
  for (auto it = m_projects.begin(); it != m_projects.end(); ++it)
  {
    seerep_core_msgs::ProjectInfo projectinfo;
    projectinfo.name = it->second->getName();
    projectinfo.uuid = it->first;
    projectinfo.frameId = it->second->getFrameId();
    projectinfo.version = it->second->getVersion();
    projectinfo.geodetCoords = it->second->getGeodeticCoordinates();

    projectInfos.push_back(projectinfo);
  }

  return projectInfos;
}

void Core::addDataset(const seerep_core_msgs::DatasetIndexable& dataset)
{
  auto project = findProject(dataset.header.uuidProject);

  project->second->addDataset(dataset);
}

void Core::addLabels(const seerep_core_msgs::Datatype& datatype,
                     const std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>&
                         labelWithInstancePerCategory,
                     const boost::uuids::uuid& msgUuid, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);

  project->second->addLabels(datatype, labelWithInstancePerCategory, msgUuid);
}

void Core::addTF(const geometry_msgs::TransformStamped& tf, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);
  project->second->addTF(tf);
}

void Core::addCameraIntrinsics(const seerep_core_msgs::camera_intrinsics& ci, const boost::uuids::uuid& projectuuid)
{
  auto project = findProject(projectuuid);
  project->second->addCameraIntrinsics(ci);
}

std::optional<seerep_core_msgs::camera_intrinsics>
Core::getCameraIntrinsics(const seerep_core_msgs::camera_intrinsics_query& ci_query)
{
  try
  {
    auto project = findProject(ci_query.uuidProject);
    return project->second->getCameraIntrinsics(ci_query.uuidCameraIntrinsics);
  }
  catch (const std::runtime_error& e)
  {
    return std::nullopt;
  }
}

bool Core::checkCameraIntrinsicsExists(const seerep_core_msgs::camera_intrinsics_query& ci_query)
{
  try
  {
    auto project = findProject(ci_query.uuidProject);
    return project->second->checkCameraIntrinsicsExists(ci_query.uuidCameraIntrinsics);
  }
  catch (const std::runtime_error& e)
  {
    return false;
  }
}

std::optional<geometry_msgs::TransformStamped> Core::getTF(const seerep_core_msgs::QueryTf& transformQuery)
{
  try
  {
    auto project = findProject(transformQuery.project);
    return project->second->getTF(transformQuery);
  }
  catch (const std::runtime_error& e)
  {
    return std::nullopt;
  }
}

std::vector<std::string> Core::getFrames(const boost::uuids::uuid& projectuuid)
{
  try
  {
    auto project = findProject(projectuuid);
    return project->second->getFrames();
  }
  catch (const std::runtime_error& e)
  {
    return {};
  };
}

std::shared_ptr<std::mutex> Core::getHdf5FileMutex(const boost::uuids::uuid& projectuuid)
{
  try
  {
    auto project = findProject(projectuuid);
    return project->second->getHdf5FileMutex();
  }
  catch (const std::runtime_error& e)
  {
    return nullptr;
  }
}
std::shared_ptr<HighFive::File> Core::getHdf5File(const boost::uuids::uuid& projectuuid)
{
  try
  {
    auto project = findProject(projectuuid);
    return project->second->getHdf5File();
  }
  catch (const std::runtime_error& e)
  {
    return nullptr;
  }
}

std::unordered_map<boost::uuids::uuid, std::shared_ptr<seerep_core::CoreProject>, boost::hash<boost::uuids::uuid>>::iterator
Core::findProject(const boost::uuids::uuid& projectuuid)
{
  auto project = m_projects.find(projectuuid);
  if (project != m_projects.end())
  {
    return project;
  }
  else
  {
    throw std::runtime_error("project " + boost::lexical_cast<std::string>(projectuuid) + "does not exist!");
  }
}

seerep_core_msgs::QueryResult Core::getDatasetFromAllProjects(seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;
  for (auto& it : m_projects)
  {
    auto dataset = it.second->getDataset(query);
    addDatasetToResult(dataset, result);
  }
  return checkSize(result, query.maxNumData);
}
seerep_core_msgs::QueryResult Core::getDatasetFromSpecificProjects(seerep_core_msgs::Query& query)
{
  seerep_core_msgs::QueryResult result;
  for (auto projectuuid : query.projects.value())
  {
    auto project = findProject(projectuuid);

    auto dataset = project->second->getDataset(query);
    addDatasetToResult(dataset, result);
  }
  return checkSize(result, query.maxNumData);
}

seerep_core_msgs::QueryResult Core::checkSize(const seerep_core_msgs::QueryResult& queryResult, uint maxNum)
{
  seerep_core_msgs::QueryResult queryResultFiltered(queryResult);

  uint64_t overallResultSize = 0;
  for (auto& queryResultProject : queryResultFiltered.queryResultProjects)
  {
    overallResultSize += queryResultProject.dataOrInstanceUuids.size();
  }

  if (maxNum > 0 && overallResultSize > maxNum)
  {
    float factor = maxNum / (float)overallResultSize;
    for (auto& queryResultProject : queryResultFiltered.queryResultProjects)
    {
      queryResultProject.dataOrInstanceUuids =
          std::vector<boost::uuids::uuid>(queryResultProject.dataOrInstanceUuids.begin(),
                                          queryResultProject.dataOrInstanceUuids.begin() +
                                              (int)std::round(queryResultProject.dataOrInstanceUuids.size() * factor));
    }
  }
  return queryResultFiltered;
}

void Core::addDatasetToResult(seerep_core_msgs::QueryResultProject& dataset, seerep_core_msgs::QueryResult& result)
{
  if (!dataset.dataOrInstanceUuids.empty())
  {
    result.queryResultProjects.push_back(dataset);
  }
}

seerep_core_msgs::AabbTime Core::getOverallTimeInterval(boost::uuids::uuid uuid,
                                                        std::vector<seerep_core_msgs::Datatype> datatypes)
{
  auto project = findProject(uuid);
  return project->second->getTimeBounds(datatypes);
}

seerep_core_msgs::AABB Core::getOverallBound(boost::uuids::uuid uuid, std::vector<seerep_core_msgs::Datatype> datatypes)
{
  auto project = findProject(uuid);
  return project->second->getSpatialBounds(datatypes);
}

std::unordered_set<std::string> Core::getAllCategories(boost::uuids::uuid uuid,
                                                       std::vector<seerep_core_msgs::Datatype> datatypes)
{
  auto project = findProject(uuid);
  return project->second->getAllCategories(datatypes);
}

std::unordered_set<std::string>
Core::getAllLabels(boost::uuids::uuid uuid, std::vector<seerep_core_msgs::Datatype> datatypes, std::string category)
{
  auto project = findProject(uuid);
  return project->second->getAllLabels(datatypes, category);
}

void Core::checkForOntologyConcepts(seerep_core_msgs::Query& query)
{
  if (query.ontologyURI)
  {
    std::unordered_map<std::string, std::string> label2ConceptCache;
    std::string concept;
    for (auto& category : query.label.value())
    {
      // only iterate over the initial size of the vector
      auto startSizeLabels = category.second.size();
      for (int i = 0; i < startSizeLabels; i++)
      {
        std::string& label = category.second.at(i);
        // check if label is NOT URL (ontology concept)
        if (!std::regex_match(label,
                              std::regex("^(https?:\\/\\/)?([\\da-z\\.-]+)\\.([a-z\\.]{2,6})([\\/\\w \\.-]*)*\\/?$")))
        {
          auto conceptCached = label2ConceptCache.find(label);
          if (conceptCached != label2ConceptCache.end())
          {
            concept = conceptCached->second;
          }
          else
          {
            concept = translateLabelToOntologyConcept(label, query.ontologyURI.value());
            label2ConceptCache.emplace(label, concept);
          }
          // add the concept to the vector so that the query will check for the initial label and also for the found concept
          category.second.push_back(concept);
        }
      }
    }
  }
}

size_t Core::writeCallback(void* contents, size_t size, size_t nmemb, std::string* output)
{
  size_t totalSize = size * nmemb;
  output->append((char*)contents, totalSize);
  return totalSize;
}

std::string Core::translateLabelToOntologyConcept(const std::string& label, const std::string& ontologyURI)
{
  std::string concept = label;

  std::string sparqlQuery = std::string("PREFIX so: <http://schema.org/>\nPREFIX skos: "
                                        "<http://www.w3.org/2004/02/skos/core#>\n\nSELECT ?c ?l\nWHERE "
                                        "{\n  {\n    ?c skos:prefLabel \"") +
                            label +
                            std::string("\"@en.\n  }\n  UNION {\n    ?c skos:altLabel ?l FILTER (str(?l) = \"") +
                            label + std::string("\").\n  }\n}");

  auto response = performCurl(sparqlQuery, ontologyURI);

  if (response)
  {
    auto conceptExtracted = extractConceptsFromJson(response.value(), "c");
    if (conceptExtracted)
    {
      concept = conceptExtracted.value().at(0);
    }
  }

  return concept;
}

std::optional<std::string> Core::performCurl(std::string query, std::string url)
{
  CURL* curl = curl_easy_init();

  if (curl)
  {
    // Set the request parameters
    std::string postData = std::string("query=").append(curl_easy_escape(curl, query.c_str(), query.length()));

    // Set the HTTP POST headers
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Accept: application/sparql-results+json");

    // Set the request options
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);

    // Response string to store the query result
    std::string response;
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    // Perform the request
    CURLcode res = curl_easy_perform(curl);

    // Check for errors
    if (res != CURLE_OK)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::error) << "curl failed: " << curl_easy_strerror(res) << std::endl;
      return std::nullopt;
    }

    // Clean up
    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    return response;
  }

  return std::nullopt;
}

std::optional<std::vector<std::string>> Core::extractConceptsFromJson(std::string json, std::string conceptVariableName)
{
  Json::Value root;
  Json::Reader reader;
  bool parsingSuccessful = reader.parse(json.c_str(), root);
  std::vector<std::string> concepts;

  if (parsingSuccessful)
  {
    try
    {
      for (auto binding : root.get("results", "").get("bindings", ""))
      {
        concepts.push_back(binding.get(conceptVariableName, "").get("value", "").asString());
      }
    }
    catch (...)
    {
      BOOST_LOG_SEV(m_logger, boost::log::trivial::error) << "Couldn't extract concept from json.";
      return std::nullopt;
    }
  }
  if (concepts.size() > 0)
  {
    return concepts;
  }
  else
  {
    return std::nullopt;
  }
}

void Core::getConceptsViaSparqlQuery(const seerep_core_msgs::SparqlQuery& sparqlQuery, const std::string& ontologyURI,
                                     std::optional<std::unordered_map<std::string, std::vector<std::string>>> label)
{
  std::optional<std::string> sparqlResult = performCurl(sparqlQuery.sparql, ontologyURI);

  if (sparqlResult)
  {
    std::optional<std::vector<std::string>> concepts =
        extractConceptsFromJson(sparqlResult.value(), sparqlQuery.variableNameOfConcept);

    if (concepts)
    {
      label = std::unordered_map<std::string, std::vector<std::string>>();
      label.value().emplace(sparqlQuery.category, concepts.value());
    }
  }
}

} /* namespace seerep_core */
